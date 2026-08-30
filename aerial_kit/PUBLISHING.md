# Publishing aerial-kit to PyPI

Publishing is tag-triggered, not commit-triggered: pushing commits to `main` does
**not** touch PyPI. A new version goes out only when you push a tag matching
`aerial-kit-v*`, via `.github/workflows/publish-aerial-kit.yml`.

## One-time setup (required before the first tag-triggered publish)

This workflow uses PyPI's **Trusted Publishing** (OIDC) instead of an API token, so
there's no secret to store in GitHub. You (the PyPI project owner) need to register this
repo's workflow as a trusted publisher once:

1. Go to https://pypi.org/manage/project/aerial-kit/settings/publishing/
2. Add a new trusted publisher:
   - Owner: `RawFish69`
   - Repository name: `aerial-kit`
   - Workflow filename: `publish-aerial-kit.yml`
   - Environment name: leave blank (the workflow doesn't declare one)
3. Save.

After that, this repo's GitHub Actions can publish without any API key ever touching
GitHub. (`aerial-kit 0.1.0` itself was published manually with an API token before this
workflow existed -- that token is no longer needed for future releases.)

## Cutting a release

1. Bump the version in `pyproject.toml` (`[project] version = "..."`), following semver.
2. Update `plans/PROGRESS.md`'s log with what changed (standard practice in this repo).
3. Commit and push to `main` as normal -- this does **not** publish anything by itself.
4. Tag and push the tag:
   ```bash
   git tag aerial-kit-v0.1.1
   git push origin aerial-kit-v0.1.1
   ```
5. The workflow builds the sdist/wheel, checks the tag version matches
   `pyproject.toml`'s version (fails fast if they disagree), and publishes to PyPI.
6. Watch the run under the repo's Actions tab; verify at
   https://pypi.org/project/aerial-kit/#history once it completes.

Version numbers on PyPI can never be reused, even if yanked -- double-check
`pyproject.toml`'s version before tagging.
