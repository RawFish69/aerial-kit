#!/usr/bin/env node
/*
 * The AerialKit bridge.
 *
 * A browser cannot open a TCP socket and cannot spawn a process, so the two
 * things a person most often wants — talk to the firmware's own simulator, or
 * to a board already claimed by something else — need a helper. This is it.
 *
 * It is a byte pipe and nothing else. It does not parse frames, does not know
 * what a parameter is, and must never learn. The moment it starts interpreting
 * the protocol there are two implementations of it and one is always behind,
 * which is the failure mode every "small helper" of this kind eventually has.
 * It also means this file never needs updating when the firmware's protocol
 * changes, which is the property that makes it trustworthy.
 *
 * Two rules are enforced here rather than left to the person reading the docs:
 *
 *  - **It binds to loopback only.** The bridge is an unauthenticated byte pipe
 *    into a flight controller. On a LAN it is a remote-control port for
 *    anyone who can reach it. Binding elsewhere needs `--expose`, which
 *    prints what it is about to do first.
 *  - **It checks the Origin header.** A WebSocket connection is not subject to
 *    the same-origin policy, so without this check *any* page a person visits
 *    could open ws://127.0.0.1:8787 and start writing to the aircraft on their
 *    desk. This is not theoretical; it is the standard attack on every local
 *    helper of this shape. Only pages served from this machine, and callers
 *    that are not browsers at all, are allowed through.
 */

import { createServer } from 'node:http';
import { spawn } from 'node:child_process';
import { WebSocketServer } from 'ws';
import process from 'node:process';

const USAGE = `
AerialKit bridge — bytes between a browser and a board.

  node bridge/index.js --sim [path]        the firmware's host simulator
  node bridge/index.js --stdio <cmd...>    any program that talks on stdin/stdout
  node bridge/index.js --tcp host:port     a board reachable over the network
  node bridge/index.js --serial <path>     a serial port (needs the optional
                                           \`serialport\` package)

Options
  --port <n>      port to serve on (default 8787)
  --host <addr>   address to bind (default 127.0.0.1; anything else needs --expose)
  --expose        bind a non-loopback address. Reads the warning and asks again.
  --origin <o>    allow one extra page origin, repeatable

The page connects to ws://127.0.0.1:<port>/ak
`.trim();

/** Where the simulator lands when the tree is built the usual way. */
const DEFAULT_SIM = 'build-host/aerialkit-sim';

function parseArgs(argv) {
  const options = {
    source: null,
    command: null,
    args: [],
    tcp: null,
    serial: null,
    port: 8787,
    host: '127.0.0.1',
    expose: false,
    extraOrigins: [],
  };

  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i];
    const next = () => {
      const value = argv[++i];
      if (value === undefined) throw new Error(`${arg} needs a value`);
      return value;
    };

    if (arg === '--sim') {
      options.source = 'sim';
      // The path is optional: the next token is only a path if it is not
      // another flag, so `--sim --port 9000` does what it looks like.
      const candidate = argv[i + 1];
      options.command = candidate !== undefined && !candidate.startsWith('--') ? argv[++i] : DEFAULT_SIM;
    } else if (arg === '--stdio') {
      options.source = 'stdio';
      const rest = argv.slice(i + 1);
      if (rest.length === 0) throw new Error('--stdio needs a command');
      options.command = rest[0];
      options.args = rest.slice(1);
      i = argv.length;
    } else if (arg === '--tcp') {
      options.source = 'tcp';
      options.tcp = next();
    } else if (arg === '--serial') {
      options.source = 'serial';
      options.serial = next();
    } else if (arg === '--port') {
      options.port = Number(next());
    } else if (arg === '--host') {
      options.host = next();
    } else if (arg === '--expose') {
      options.expose = true;
    } else if (arg === '--origin') {
      options.extraOrigins.push(next());
    } else if (arg === '--help' || arg === '-h') {
      console.log(USAGE);
      process.exit(0);
    } else {
      throw new Error(`unknown option ${arg}`);
    }
  }

  if (options.source === null) throw new Error('choose a source: --sim, --stdio, --tcp or --serial');
  if (!Number.isInteger(options.port) || options.port <= 0 || options.port > 65535) {
    throw new Error(`--port must be a port number, not ${options.port}`);
  }
  return options;
}

/**
 * The source, as a thing with `write` and events.
 *
 * Every source is reduced to the same shape here so the rest of the file —
 * the websocket side, the single-client rule, the teardown — is written once.
 */
async function openSource(options) {
  if (options.source === 'tcp') {
    const [host, port] = splitHostPort(options.tcp);
    const { connect } = await import('node:net');
    const socket = connect({ host, port });
    await new Promise((resolve, reject) => {
      socket.once('connect', resolve);
      socket.once('error', reject);
    });
    socket.setNoDelay(true);
    return {
      label: `tcp ${host}:${port}`,
      write: (bytes) => socket.write(bytes),
      onData: (handler) => socket.on('data', handler),
      onClose: (handler) => socket.on('close', handler),
      close: () => socket.destroy(),
    };
  }

  if (options.source === 'serial') {
    let SerialPort;
    try {
      ({ SerialPort } = await import('serialport'));
    } catch {
      throw new Error(
        'the serial source needs the optional `serialport` package:\n' +
          '  cd apps/configurator/bridge && npm install serialport\n' +
          'Or use --stdio with any program that can open the port.',
      );
    }
    const port = new SerialPort({ path: options.serial, baudRate: 115200 });
    await new Promise((resolve, reject) => {
      port.once('open', resolve);
      port.once('error', reject);
    });
    return {
      label: `serial ${options.serial} at 115200`,
      write: (bytes) => port.write(bytes),
      onData: (handler) => port.on('data', handler),
      onClose: (handler) => port.on('close', handler),
      close: () => port.close(() => {}),
    };
  }

  // --sim and --stdio are the same thing: a child process with bytes on its
  // standard streams. The simulator is a default path, not a special case.
  const child = spawn(options.command, options.args, { stdio: ['pipe', 'pipe', 'inherit'] });
  await new Promise((resolve, reject) => {
    child.once('spawn', resolve);
    child.once('error', reject);
  });
  return {
    label: `${options.command}${options.args.length > 0 ? ' ' + options.args.join(' ') : ''}`,
    write: (bytes) => child.stdin.write(bytes),
    onData: (handler) => child.stdout.on('data', handler),
    onClose: (handler) => child.on('exit', (code, signal) => handler(`exited (${signal ?? code})`)),
    close: () => child.kill(),
  };
}

function splitHostPort(text) {
  const at = text.lastIndexOf(':');
  if (at <= 0) throw new Error(`--tcp wants host:port, not ${text}`);
  const port = Number(text.slice(at + 1));
  if (!Number.isInteger(port)) throw new Error(`--tcp has a bad port: ${text}`);
  return [text.slice(0, at), port];
}

function isLoopback(host) {
  return host === '127.0.0.1' || host === 'localhost' || host === '::1' || host === '[::1]';
}

/**
 * Whether a page is allowed to open this bridge.
 *
 * A browser sends `Origin`. A page on this machine — the dev server, a static
 * build served locally — is fine. Anything else on the internet is not, and
 * neither is a page that reached loopback by DNS rebinding, which arrives with
 * a public hostname in `Origin` and is exactly why this check is here.
 *
 * A caller with no `Origin` at all is not a browser: a CLI, a test, another
 * script. Those are allowed, because they can already read the device directly
 * and this check is not what stops them.
 */
function originAllowed(origin, extra) {
  if (origin === undefined || origin === '') return true;
  if (origin === 'null') return true; // a local file:// page
  let url;
  try {
    url = new URL(origin);
  } catch {
    return false;
  }
  if (isLoopback(url.hostname)) return true;
  return extra.includes(origin);
}

async function main() {
  const options = parseArgs(process.argv.slice(2));

  if (!isLoopback(options.host) && !options.expose) {
    console.error(
      `refusing to bind ${options.host}.\n\n` +
        'The bridge is an unauthenticated byte pipe into a flight controller. On a\n' +
        'network it is a remote control for the aircraft on your desk, for anyone who\n' +
        'can reach the port — there is no password, and the protocol has no notion of\n' +
        'one. If you have thought about that and still want it, pass --expose.',
    );
    process.exit(2);
  }

  const source = await openSource(options);
  console.log(`source: ${source.label}`);

  const server = createServer((request, response) => {
    // A plain HTTP request to this port is someone who has not read the usage.
    response.writeHead(426, { 'content-type': 'text/plain', connection: 'close' });
    response.end('this is a websocket bridge; connect to ws://host:port/ak\n');
    void request;
  });

  const wss = new WebSocketServer({ server, path: '/ak' });
  let client = null;

  wss.on('connection', (socket, request) => {
    const origin = request.headers.origin;
    if (!originAllowed(origin, options.extraOrigins)) {
      console.warn(`refused a page from ${origin}`);
      socket.close(1008, 'this bridge only serves pages on this machine');
      return;
    }

    // One client at a time, and this is not a limitation to work around. The
    // protocol correlates a reply to its request by the command byte alone, so
    // two clients writing at once would each receive the other's answers and
    // both would be wrong in a way neither could detect.
    if (client !== null) {
      socket.close(1013, 'another page is already connected to this board');
      return;
    }

    client = socket;
    console.log(`connected: ${origin ?? 'a non-browser caller'}`);

    const onData = (chunk) => {
      if (socket.readyState === socket.OPEN) socket.send(chunk, { binary: true });
    };
    source.onData(onData);

    socket.on('message', (data, isBinary) => {
      // Text frames are a mistake worth reporting rather than forwarding: the
      // board would see the wrong bytes and the person would see a checksum
      // error with no explanation.
      if (!isBinary) {
        console.warn('refused a text frame; the protocol is bytes');
        return;
      }
      source.write(data);
    });

    socket.on('close', () => {
      if (client === socket) client = null;
      console.log('disconnected');
    });

    socket.on('error', (error) => console.warn(`socket: ${error.message}`));
  });

  source.onClose((reason) => {
    console.log(`source closed: ${reason}`);
    if (client !== null) client.close(1011, `the board went away: ${reason}`);
    client = null;
  });

  server.listen(options.port, options.host, () => {
    console.log(`listening on ws://${options.host}:${options.port}/ak`);
    if (!isLoopback(options.host)) {
      console.log('');
      console.log('  This bridge is reachable from the network. Anyone who can open');
      console.log('  that port can write to the board. Ctrl-C stops it.');
      console.log('');
    }
  });

  const shutdown = () => {
    console.log('\nclosing');
    if (client !== null) client.close(1001, 'the bridge is shutting down');
    source.close();
    wss.close();
    server.close(() => process.exit(0));
    // A child that ignores SIGTERM should not hold the terminal hostage.
    setTimeout(() => process.exit(0), 1000).unref();
  };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}

main().catch((error) => {
  console.error(`\n${error.message}\n`);
  process.exit(1);
});
