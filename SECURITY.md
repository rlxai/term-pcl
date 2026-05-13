# Security Policy

## Supported versions

Security fixes are provided for the latest released version and the current development branch.

## Reporting a vulnerability

Please report security issues privately to Robolabs AI at contact@robolabs.ai.

Include enough detail to reproduce the issue when possible:

- `term-pcl` version or commit
- Operating system and distribution
- Input file type and a minimal reproducer if it can be shared safely
- Command line used
- Observed behavior and expected safe behavior

Please do not disclose vulnerabilities publicly until the issue has been investigated and a fix or mitigation is available.

## Security scope

Security-relevant reports include:

- malformed point-cloud or `.termcloud` inputs that crash the process
- parser behavior that causes excessive resource use
- unsafe filesystem behavior while indexing or checking files
- corrupted index/chunk data that is accepted silently

General support requests and non-security bugs should use the public issue tracker once the project repository is public.
