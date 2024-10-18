
## libGL error
```shell
libGL error: MESA-LOADER: failed to open iris: /usr/lib/dri/iris_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)
```
solution in [StackOverflow](https://stackoverflow.com/questions/72110384/libgl-error-mesa-loader-failed-to-open-iris): add the following to `~/.bashrc` or `~/.zshrc` (if using zsh).
```shell
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
```