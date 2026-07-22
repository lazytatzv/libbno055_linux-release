# Release Guide (Maintainer Only)

This document serves as a cheat sheet for the maintainer on how to release a new version of `libbno055-linux` to both ROS 2 (`rosdistro`) and the Ubuntu PPA.

## 1. Bump the Version

Before releasing, you must increment the version number in all package managers and documentation.

1. **Update `CMakeLists.txt`**:
   ```cmake
   project(libbno055_linux VERSION X.Y.Z LANGUAGES CXX)
   ```
2. **Update `package.xml`**:
   ```xml
   <version>X.Y.Z</version>
   ```
3. **Update `vcpkg.json`**:
   ```json
   "version": "X.Y.Z"
   ```
4. **Update `conanfile.py`**:
   ```python
   version = "X.Y.Z"
   ```
5. **Update `setup.py`**:
   ```python
   version="X.Y.Z"
   ```
6. **Update `rust/Cargo.toml`**:
   ```toml
   version = "X.Y.Z"
   ```
7. **Update `CHANGELOG.md`**:
   Add a new section for `## [X.Y.Z] - YYYY-MM-DD` detailing the changes.
8. **Update `debian/changelog`**:
   ```bash
   dch -v X.Y.Z-1 "Release version X.Y.Z"
   ```
   *(Or edit manually following the existing format).*

Commit and push these changes, then cut a new Git tag:
```bash
git add .
git commit -m "chore: bump version to X.Y.Z"
git push
git tag -a vX.Y.Z -m "Release vX.Y.Z"
git push origin vX.Y.Z
```

---

## 2. ROS 2 Release (via Bloom)

To release to the official ROS 2 apt repositories, we use `bloom-release`. This automatically opens a Pull Request to `ros/rosdistro`.

**Prerequisites**: Make sure you have a GitHub Personal Access Token configured for `bloom`.

### Releasing for Humble
```bash
bloom-release --rosdistro humble --track humble libbno055_linux
```

### Releasing for Jazzy
```bash
bloom-release --rosdistro jazzy --track jazzy libbno055_linux
```

### Releasing for Kilted
```bash
bloom-release --rosdistro kilted --track kilted libbno055_linux
```

*Note: If `bloom` warns that a pull request already exists, ensure you delete the previous `bloom-libbno055_linux-X` branch from your GitHub fork of `rosdistro` before running.*

---

## 3. Ubuntu PPA Release (Standalone C++)

To distribute the pure C++ library to non-ROS users via `apt`, upload the source package to your Launchpad PPA.

**Prerequisites**: You must have `devscripts`, `debhelper`, and `dput` installed, and your GPG key configured for Launchpad.

1. **Build the Source Package**:
   ```bash
   # This will build the source package and sign it with your GPG key
   debuild -S -sa
   ```

2. **Upload to Launchpad**:
   ```bash
   # Move up one directory where the generated .changes file is located
   cd ..
   
   # Replace `lazytatzv/bno055` with your actual PPA name if different
   dput ppa:lazytatzv/bno055 libbno055-linux_X.Y.Z-1_source.changes
   ```

Launchpad will then build the `.deb` files for various architectures in the cloud and publish them to your PPA.

---

## 4. Automated `crates.io` Release (via GitHub Actions)

Pushing a Git tag matching `v*` (e.g. `git push origin v1.5.0`) triggers the automated release workflow in `.github/workflows/release.yml`.

**Prerequisites**:
1. Obtain an API Token from [crates.io](https://crates.io/settings/tokens).
2. Store the token as a GitHub Secret in your repository:
   - Go to **Settings ➔ Secrets and variables ➔ Actions**
   - Add a New repository secret named **`CRATES_IO_TOKEN`**

Once set up, whenever a tag is pushed:
* GitHub Release will be automatically generated with release notes.
* CPack `.deb` and `.tar.gz` packages will be built and attached.
* The `libbno055` Rust crate will be automatically published to `crates.io`.

