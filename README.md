# MARS - ROS

ROS packages for the MARS club

## Cloning and Pulling

> Because this repository uses submodules, care must be taken when pulling/pushing 

To clone this repository, you need to add the `--recursive` flag

```bash
git clone --recursive https://github.com/hanzh713/mars-ros
```

If you forgot the `--recursive` flag, you need to run submodule init manually after cloning

```bash
git submodule update --init
```

To pull changes from the remote, you need to run two commands

```bash
git pull
git submodule update --init
```

For more information about git submodules, refer to the git handbook: https://git-scm.com/book/en/v2/Git-Tools-Submodules

## Note for development

You need to have catkin_tools installed. You can use `sudo apt install python-catkin-tools` to install catkin_tools

When developing for specific packages, you do not need to compile all other packages. You can specify the name of the packages you want to build as arguments to `catkin build`. For example, 

```bash
catkin build segmentation processing
```

will only build `processing` and `segmentation` packages. For other advanced build options, refer to the documentation: https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html

### VSCode Remote Development on Jetson

[VSCode remote development](https://code.visualstudio.com/docs/remote/ssh) works well. However, for the Python/C++ language extension to work, you need to tweak some settings.

#### Python:

Microsoft Python Language Server does not support arm64. Therefore, you need to add `"python.jediEnabled": true` to the workspace settings

#### C++:

Microsoft C++ extension does not support arm64. You need to install the `vscode-clangd` plugin from llvm instead. Follow the plugin README to get started. 

However, it currently limits to providing intellisense etc. for one package at a time. To create `compile_commands.json`, run 

```bash
catkin build packagename -DCMAKE_EXPORT_COMPILE_COMMANDS=1
ln -s devel/packagename/compile_commands.json compile_commands.json
```

reload window to activate the extension

## Compile all packages

If you need to compile all packages, refer to the [compile guide](./CompileGuide.md)