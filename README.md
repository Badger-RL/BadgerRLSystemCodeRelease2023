
# Badger RL README

This is the BadgerRL internal fork of BHumanCodeRelease.

## Setup instructions

This codebase is compatible with both MacOS and Ubuntu 22.04, though the experience tends to be best on Ubuntu 22.04.  

To setup up this repository, first install the dependencies via apt as listed [here](https://wiki.b-human.de/coderelease2022/getting-started/), then run:


````
git clone --recursive https://github.com/Badger-RL/BadgerRLSystem2022.git

````

There are MacOS and Ubuntu specific instructions for generating and compiling the code [here](https://wiki.b-human.de/coderelease2022/getting-started/).


To run the simulator after compiling it(example provided for Ubuntu), you can run the following command from the top level directory of the project:


````
./Build/Linux/SimRobot/Develop/SimRobot
````

# B-Human Code Release README

This is the official 2022 B-Human code release. Documentation can be found in our [public wiki](https://wiki.b-human.de/coderelease2022/).

B-Human is a research project. We provide the software as is. We release it as a snapshot of our internal repository on a yearly basis. You can build on this software, but you have to make do with the documentation we released. You may report bugs or problems with newer releases of operating systems or the tools used by opening a GitHub issue. We might give hints on how to solve these problems if we also ran into them since the code release. Do not write emails to the team or any of its members directly. They will not be answered.

Before cloning this repository, you must read our [license terms](License.md) and the [instructions in our wiki](https://wiki.b-human.de/coderelease2022/getting-started/).

Previous code releases are tagged with "coderelease&lt;year&gt;", where &lt;year&gt; is the year in which the code was released (starting with 2013).
