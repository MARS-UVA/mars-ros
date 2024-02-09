Iriutils library                         {#mainpage}
================

## Description
This set of tools provide basic functionalities that are generally needed in many
applications. These utilities include mutual exclusion objects, exceptions, threads,
logic events and also generic communication devices. 


<details><summary>Utilities description</summary>
<p>

### Mutex

Basic mutual exclusion object interface to handle multiple threads to access the
same shared resources. There exist no example of use of mutexes, but they are used
in most of the other utilities, so check them for examples of use.

### Exceptions

Generic exceptions to be used as a base for all other exceptions. It provides some
basic error message handling, and each inherited exception may add as much
information as needed. There is no examples of use of exceptions, but it is used
in most of other utilities, so check them for examples of use.

### Threadserver and Threads

This utility provides a simple and easy to use interface to threads, isolating the
low level details from the end user. The threads may be used directly or else
through a thread server which allows to create and handle as many threads as
necessary without direct access to the threads themselves. In this case, each
thread is assigned a unique identifier which is used to identify it in the
server.

There exist examples of both threads being used directly (test_threads) and also
threads being used through the thread server (test_threadserver). These examples
provide the necessary information to easily set up an application with multiple
threads.

### Eventserver and Events

This utility provides a simple and easy way to use asynchronous notifications
between threads, allowing them to wait on several heterogeneous conditions without
wasting CPU time. The events may be used directly or else through an event server,
which allows to create and handle as many events as needed. In this case each event
is assigned a unique identifier which is used to identify it in the server.

There exist examples of both events being used directly (test_events) and also
events being used through the event server (test_eventserver). These examples
provide the necessary information to easily set up an application with multiple
events.

There exist also an other example that uses both events and threads using their
servers (test_bothservers), which gives a good example on how to use them
together.

### Log

This utility provides a simple way to log information into files. Each object is
associated to single file, and all the logged messages include a time stamp of
the log time.

### Time

This utility provides a simple and easy way to use time. Each CTime object has
got a time and using the class members and operations it is possible to get
time in seconds, milliseconds or C/system types such as timespec, or time_t;
set a time, sum, difference or average times, do time comparisons or print
time in human readable format.

There exist an example which uses all of class methods. Also in a bad use
which reach to an exception.

</p>
</details>

## Installation

* Add the labrobotica repository if it is not already added:

Run the commands on _add repository_ and _add key_ from [labrobotica_how_to installation](https://gitlab.iri.upc.edu/labrobotica/labrobotica_how_to/-/blob/master/README.md#installation)

* Install the package:

``` sudo apt update && sudo apt install iri-iriutils-dev ```

## Scripts

The following scripts located in `./scripts/` are provided:

* **add_lib_to_ld_config.sh**: To add the library path to ldconfig
  - Usage:

        ./add_lib_to_ld_config.sh -l <library> [-p]
          -l  specify library name, for example iriutils"
          -p  specify if installed from package. Optional."

* **remove_lib_from_ld_config.sh**: To remove the library path from ldconfig
  - Usage:

        ./remove_lib_from_ld_config.sh -l <library> [-p]
          -l  specify library name, for example iriutils"
          -p  specify if installed from package. Optional."

These scripts will be added to the PATH variable after installation.

## Disclaimer  

Copyright (C) 2009-2018 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Mantainer IRI labrobotics (labrobotica@iri.upc.edu)

This package is distributed in the hope that it will be useful, but without any warranty. It is provided "as is" without warranty of any kind, either expressed or implied, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose. The entire risk as to the quality and performance of the program is with you. should the program prove defective, the GMR group does not assume the cost of any necessary servicing, repair  or correction.

In no event unless required by applicable law the author will be liable to you for damages, including any general, special, incidental or consequential damages arising out of the use or inability to use the program (including but not limited to loss of data or data being rendered inaccurate or losses sustained by you or third parties or a failure of the program to operate with any other programs), even if the author has been advised of the possibility of such damages.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

## For developers

<details><summary>click here</summary>
<p>

## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [doxygen](http://www.doxygen.org "Doxygen's Homepage") and [graphviz](http://www.graphviz.org "Graphviz's Homepage") to generate the documentation.
* stdc++ and pthread libraries.

Under linux all of these utilities are available in ready-to-use packages.

Under MacOS most of the packages are available via [fink](http://www.finkproject.org/ "Fink's Homepage")

## Compilation and installation from source

Clone this repository and create a build folder inside:

``` mkdir build ```

Inside the build folder execute the following commands:

``` cmake .. ```

The default build mode is DEBUG. That is, objects and executables include debug information.

The RELEASE build mode optimizes for speed. To build in this mode execute instead
``` cmake .. -DCMAKE_BUILD_TYPE=RELEASE ```

The release mode will be kept until next time cmake is executed.

``` make -j $(nproc)``` 

In case no errors are reported, the generated libraries (if any) will be located at the
_lib_ folder and the executables (if any) will be located at the _bin_ folder.

In order to be able to use the library, it it necessary to copy it into the system.
To do that, execute

``` make install ```

as root and the shared libraries will be copied to */usr/local/lib/iri/iriutils* directory
and the header files will be copied to */usr/local/include/iri/iriutils* dierctory. At
this point, the library may be used by any user.

To remove the library from the system, exceute

``` make uninstall ```

as root, and all the associated files will be removed from the system.

To generate the documentation execute the following command:

``` make doc ```

## How to use it

To use this library in an other library or application, in the CMakeLists.txt file, first it is necessary to locate if the library has been installed or not using the following command

``` FIND_PACKAGE(iriutils REQUIRED) ```

In the case that the package is present, it is necessary to add the header files directory to the include directory path by using

``` INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR}) ```

and it is also necessary to link with the desired libraries by using the following command

``` TARGET_LINK_LIBRARIES(<executable name> ${iriutils_LIBRARY}) ```

## Examples

There are some examples:
 * _test\_both_: An example of how two threads communicate through events.
 * _test\_events_: It shows the basic operation with events and the most important feature, which is the wait() function.
 * _test\_eventserver_: It shows how to handle multiple events using the CEventServer class.
 * _test\_logs_: It shows the differents log options provided by the class which include logging.
 * _test\_threads_: It shows how to use the CThread class.
 * _test\_threadserver_: It shows how to handle multiple threads using the CThreadServer class.
 * _test\_time_: It checks all functions in CTime class.


</p>
</details>

