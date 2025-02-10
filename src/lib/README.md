# AutoHYU-LIB

**Adding Submodules**

The AutoHYU-LIB project manages external libraries or other Git repositories as submodules. To add a new submodule, use the following command:

```bash
git submodule add [submodule_repository_URL]
```

This command adds the specified repository as a subdirectory in the current project and records submodule information in the `.gitmodules` file.

**Adding a Submodule with a Specific Tag**

To add a submodule and set it to a specific tag:

1. First, add the submodule:
   ```bash
   git submodule add [submodule_repository_URL]
   ```

2. Move into the submodule directory:
   ```bash
   cd [submodule_directory]
   ```

3. Check out the desired tag:
   ```bash
   git checkout [tag_name]
   ```

4. Commit the change in the main repository to link the submodule to the specified tag:
   ```bash
   cd ..
   git add [submodule_directory]
   git commit -m "Update submodule to specific tag"
   ```

This ensures that the submodule is fixed to a specific tag.

**Updating Submodules**

After cloning the project or when submodules have been updated, initialize and keep them up-to-date by running:

```bash
git submodule update --recursive --init
```

This command recursively initializes all submodules, checking out each one to the designated commit to ensure the project operates as intended. Regularly running this command is recommended to keep submodules synchronized with the latest changes.