# March
The main repository of the MARCH exoskeleton.

| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/march.svg?branch=master)](https://travis-ci.com/project-march/march) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/march.svg?branch=develop)](https://travis-ci.com/project-march/march) |

## Fixing code style
All C++ code must follow the [`roscpp_code_format`](https://github.com/davetcoleman/roscpp_code_format)
code styling rules. The rules for this format are set in the `.clang-format`
file. `clang-format` is a tool that can detect and fix these problems in your
code. Before pushing you should make sure that this is fixed, otherwise the
Travis build will fail. First you need to install `clang-format`:
```
sudo apt install clang-format
```
Then you can run `clang-format` from the root of this repository:
```
find . -name '*.h' -or -name '*.cpp' | xargs clang-format -i -style=file
```
**NOTE:** This command can make changes to your files.

If you would like to show a diff and not use `find`, install
[`clang_format_check`](https://github.com/cloderic/clang_format_check).
