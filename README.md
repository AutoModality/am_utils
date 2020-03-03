# AM Utils

A common library for sharing core code.



| Github Action  | Debian Package |
| ------------- | ------------- |
| [![Story Development](https://github.com/AutoModality/am-utils/workflows/Story%20Development/badge.svg)](https://github.com/AutoModality/am-utils/actions?query=workflow%3A%22Story+Development%22) | --  |
| [![Release Candidate](https://github.com/AutoModality/am-utils/workflows/Release%20Candidate/badge.svg)](https://github.com/AutoModality/am-utils/actions?query=workflow%3A%22Release+Candidate%22)  | [![Latest Version @ Cloudsmith](https://api-prd.cloudsmith.io/badges/version/automodality/dev/deb/ros-kinetic-am-utils/latest/d=ubuntu%252Fxenial;t=1/?render=true&badge_token=gAAAAABeEXmnoIyJ49KZBuqu7owI69o58O-pEmDK1uXKcvR-65M3zvVldA6lRgnSlbOXlwUdmAwIYqjEaAIZ_K0DXuO7agbeqnAtp1uXvuwoxD6KMqz3wYQ%3D)](https://cloudsmith.io/~automodality/repos/dev/packages/detail/deb/ros-kinetic-am-utils/latest/d=ubuntu%252Fxenial;t=1/)  |
| [![Release](https://github.com/AutoModality/am-utils/workflows/Release/badge.svg)](https://github.com/AutoModality/am-utils/actions?query=workflow%3A%22Release)  | [![Latest Version @ Cloudsmith](https://api-prd.cloudsmith.io/badges/version/automodality/release/deb/ros-kinetic-am-utils/latest/d=ubuntu%252Fxenial;t=1/?render=true&badge_token=gAAAAABeEYCXaXnvhWBdS0An17OCBntPch8LyCzJnPoQYMA4Ng8uIJnw2hvwdvv_NBqAOTu76_FBfvns72pfHrzV2xWoBFEvGA-hns2hV-4yu2BmqWtHCT0%3D)](https://cloudsmith.io/~automodality/repos/release/packages/detail/deb/ros-kinetic-am-utils/latest/d=ubuntu%252Fxenial;t=1/)|


## Run in Docker

Mounts your local clone of this repository to `/workspace` directory in a docker container.

**Prerequisites**
* Docker is installed
* This repository is cloned 
* Your current directory is the root of this repository

```
docker run -w /workspace -v `pwd`:/workspace -it docker.cloudsmith.io/automodality/dev/amros-base
debian/rules clean
debian/rules build
```

## Build

Build with debian in docker.


## Package 

Runs `Build` and packages for installation.

## Releases

See [releases](https://github.com/AutoModality/am-utils/releases) for the latest.  Follows the standard [Development Lifecycle](https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/668664251/Development+Lifecycle). 


