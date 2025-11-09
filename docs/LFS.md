### Large File Store in Repo.

## Install
- macOS: brew install git-lfs
- Debian/Ubuntu: sudo apt install git-lfs

After install, enable in the project's root directory:
git lfs install

## Track file types
Decide which file patterns should go to LFS and register them once (from repo root):
git lfs track "*.pth"
git lfs track "src/solidworks/*.mp4"

This creates/updates .gitattributes. Commit that file:
git add .gitattributes
git commit -m "Track large files with Git LFS"

Example .gitattributes entry:
*.pth filter=lfs diff=lfs merge=lfs -text

## Adding and pushing files
Add large files as usual:
git add models/big_model.pth
git commit -m "Add model"
git push origin main

Git LFS will upload actual blobs to the LFS server while Git stores pointers.

## Check LFS status and usage
- List tracked files: git lfs ls-files
- Check pointer files (if something looks like a small text pointer): git lfs pointer --file <file>
- View repo LFS usage on your Git hosting service (GitHub/GitLab).

## Resources
- https://git-lfs.github.com
- GitHub: https://docs.github.com/en/repositories/working-with-files/managing-large-files
