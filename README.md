# AVA_Control
Repository of ROS nodes composing the Control Module in the USDOT AVA Project.

To clone this repository:
```
git clone --recurse-submodules git@github.com:ava-share/AVA_Control.git
```
If you cloned this repository without the `--recurse-submodules` argument, you should initialize and update all submodules. A foolproof way to do this:
```
git submodule update --init --recursive
```

# Repository Organization
- This is a collection of [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
- `AVA_Control` directory contains most of the deployable code.
- `MPC` contains a repository to MATLAB/Simulink implementation of MPC for prototyping.
- `ros_node_stanley_control` contains a repository to a python2 implementation of the Stanley lateral control algorithm. This repository is no longer used as the code in `AVA_Control` supercedes it.
- `TAMU_vector_maps` contains directories of vector maps used by Texas A & M University (TAMU) in doing demos.

# Collaboration
1. Pull Requests (PRs) are required to push to `main`. 
2. PRs to `main` require approval from codeowners. This is to reduce the burden on one single maintainer and to ensure that the code is reviewed by someone familiar with the impacted files.
2. Please make changes to a branch that identifies yourself as the author and a meaningful name.
    - Example: ```tvidano/add_collaboration_to_readme```
3. Please use informative PR names and descriptions. They are used as the default commit message.
4. Please [squash and merge](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/about-pull-request-merges) commits to `main` so that the commit history is clean.