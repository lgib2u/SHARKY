# Security

The SHARKY dashboard can issue physical movement commands and does not include
built-in user authentication. It binds to loopback by default. Do not expose it
directly to a local network or the public internet; use an authenticated local
proxy and a restrictive firewall if remote access is genuinely required.

Keep Wi-Fi credentials, SSH details, hostnames, network addresses, environment
files, private keys, and device images out of the repository. The example
configuration uses non-working placeholders and the real local configuration is
ignored by Git.

To report a vulnerability without publishing operational details, use the
repository's private **Security → Report a vulnerability** flow on GitHub.
