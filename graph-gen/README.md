# Graph Generation

## Building 

`dune build`

## Running 

Currently just change the variables within the program, rebuild then run `dune exe ./c_grph.exe`.

:warning: *BELOW NOT SUPPORTED YET*

`dune exe ./c_grph.exe -- <height> <width> <tunnels> <depth>`

optionally add the following function to your `.bashrc | .zshrc` and run with `gexe <height> <width> <tunnels> <depth>`

```
gexe () {
    dune exe ./c_grph.exe -- "$1" "$2" "$3" "$4"
}
```
