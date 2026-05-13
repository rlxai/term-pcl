# Apt repository publishing workflow

This project produces Debian package artifacts in CI. A static apt repository can be published from those `.deb` files with standard Debian tools.

## Generate repository metadata

From a directory containing package artifacts:

```bash
mkdir -p apt-repo/pool/main/t/term-pcl
cp term-pcl_*.deb apt-repo/pool/main/t/term-pcl/
cd apt-repo
mkdir -p dists/stable/main/binary-amd64
apt-ftparchive packages pool > dists/stable/main/binary-amd64/Packages
gzip -kf dists/stable/main/binary-amd64/Packages
apt-ftparchive release dists/stable > dists/stable/Release
```

## Sign repository metadata

```bash
gpg --default-key <signing-key-id> --clearsign -o dists/stable/InRelease dists/stable/Release
gpg --default-key <signing-key-id> -abs -o dists/stable/Release.gpg dists/stable/Release
```

## Publish

Upload `apt-repo/` to a static host such as GitHub Pages or any HTTPS file host. Users can then install with the commands in `README.md` after replacing the repository URL and signing key URL.
