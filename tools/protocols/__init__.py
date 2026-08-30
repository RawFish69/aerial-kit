"""
Protocol Decoders Package
"""

from .crsf_decoder import CRSFDecoder
from .sbus_decoder import SBUSDecoder
from .ibus_decoder import IBUSDecoder

__all__ = ['CRSFDecoder', 'SBUSDecoder', 'IBUSDecoder']

# Protocol registry
PROTOCOLS = {
    'CRSF': CRSFDecoder,
    'SBUS': SBUSDecoder,
    'iBus': IBUSDecoder,
}

def get_protocol(name):
    """Get protocol decoder by name"""
    return PROTOCOLS.get(name)

def list_protocols():
    """List all available protocols"""
    return list(PROTOCOLS.keys())

