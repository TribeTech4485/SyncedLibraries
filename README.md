# IMPORTANT: NOT USABLE FOR BEFORE 2025

The reworking of CANSparkMax for the 2025 season breaks compatibility with old systems. Given the lack of back-compatability,
many other changes were made removing deprecated functions and other refactioning to imporove cleanlieness.\
Last compatable commit: [f047dbe](https://github.com/TribeTech4485/SyncedLibraries/commit/f047dbe1a0dbf6bf40d245322331d21139da2d80)

## General information

Never put anything robot specific within this folder/repo. This is suppossed to be shared between all future robots to provide more consistency within robot designs. Always put in a directory named `SyncedLibraries`

When making changes that could possibly change the output of old implimentations of this class, mark the function as `@Depricated` and make a new function. If not possible, add a javadoc to quickly explain how to update.

The SystemBases folder contains template classes for major systems to provide further continuity between future code. This is especially important to follow the instructions above. To use, extend the class and `@Override` the methods.

## To add to existing project

```shell
cd src/main/java/frc/robot/
git submodule add https://github.com/TribeTech4485/SyncedLibraries
```

And you're done!

----

Add to readmes of projects containing:

## TO SAVE

Always use the command line git to install as shown below, or it will not work:\
`git clone --recurse-submodules {URL}`

Odds are, if you are reading this because you cloned through the GUI, so use the command below while in the repo to fix:\
`git submodule update --init --recursive`

## When using

Also note the sub-repository [SyncedLibraries](https://github.com/TribeTech4485/SyncedLibraries). The README should explain the basics of how to operate.
