# ZhanfengSharing
A Repo to share stuff with Zhanfeng

**I HIGHLY RECOMMEND YOU INSTALL THIS**. It makes things colourful in the terminal :D
```Bash
pip3 install termcolor
```

To build all, use
```Bash
chmod 700 build_all.zsh # To change the permissions for the build_all.zsh 
./build_all.zsh #[param]  To build everything; param 0 to build data_structures; param 1 to build sample_server 
```

To run, open 3 command terminals and source each terminal. Run,
```Bash
ros2 run sample_server <filename>
```
in the following order
1. `server`
2. `first` *or* `second` **NOT BOTH**
3. `client`
