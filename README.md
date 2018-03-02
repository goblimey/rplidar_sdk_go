# rplidar_sdk_go
A Go wrapper for the RPLidar SDK

Seeed Studio's RPLidar is a range of devices intended for Simultaneous 
Localisation And Mapping (SLAM) applications.

Slamtech publish the RPLidar SDK, a library of software to control the RPLidar
[here](https://www.slamtec.com/en/Support/).  It's written in C++.

This software is a wrapper to allow Go applications to control an
RPLidar device using the Slamtech SDK.
At present, it's just a proof of concept that implements a few
functions, in particular startMotor and stopMotor.

The Slamtec control library is written in C++ and can be built for
a Windows, Mac or Linux environment.  To do this you need a C++
compiler and a Make tool.  The GNU group provide both of these for free.

To build the Go wrapper, you need to first download and build the Slamtec library
for your environment:

Download the library distibution zip file from the above link.

Unzip the distribution into a directory somewhere.

Unzipping it produces directories sdk and tools.

    $ cd sdk
    $ make clean
    $ make

This produces the library in sdk/output/Linux/Release/librplidar_sdk.a

Now download the Go wrapper:

    $ go get github.com/goblimey/rplidar_sdk_go

This will download the software, but the build will fail, because it needs the Slamtech library.
Copy the library from the directory shown above to the lib directory of the wrapper
distribution (src/github.com/goblimey/rplidar_sdk_go/lib in your Go project directory).
Now you can compile the demonstration program:

    $ go install github.com/goblimey/rplidar_sdk_go

Connect your rplidar scanner to your USB port and run the demo:

    $ rplidar_sdk_go

The scanner will spin for ten seconds and then the program will finish.



The Gory Details
========================

Go can only call C++ software via a C interface, which restricts what it can call.
The Slamtech library uses all sorts of clever C++ features that defeat this calling interface,
so there is a class ProxyDriver (proxy_driver.cpp)
which calls the Slamtech methods and hides the clever stuff.
The Driver class (driver.cpp) calls the proxy's methods.
The functions in cdriver.cpp provides a C interface to the driver class which the Go software can call.
In the Go world, the type Driver (in driver.go) implements a version of each of the methods in the slamtech library.

For example, in driver.go, the method StartMotor() calls the C interface method C.RPDriverStartMotor(),
which calls StartMotor() in the C++ Driver class, which calls StartMotor() in the C++ ProxyDriver class,
which calls startMotor() in the Slamtech library.

The driver.go file contains some magic to glue all this stuff together:

    // #cgo CFLAGS: -I${SRCDIR}/../include -DDEBUG -g
    // #cgo CXXFLAGS: -I${SRCDIR}/../include -DDEBUG -g
    // #cgo CPPFLAGS: -I${SRCDIR}/../include -DDEBUG -g
    // #cgo LDFLAGS: -L${SRCDIR}/../lib -lrplidar_sdk -lrt -lstdc++ -lpthread
    /*
    #include <stdio.h>
    #include <stdlib.h>
    #include "rptypes.h"
    #include "driver.h"
    */
    import "C"

Behind the scenes, the go command runs the C++ compiler, which can compile both C++ and C code.
The #cgo comment lines specify directives for the C++ compiler.
${SRCDIR} defines the current directory.
-I says where to find the header files. 

The -l option specifies libraries.
The compiler knows where to find the standard libraries
but it needs to be told where to find the Slamtech library that you built and copied..
-L says which directory to find the library. 
-l defines the library itself (-lx means the file libx.a).  

-DDEBUG turns on debugging code.  

-g tells the compiler to include symbol table data so that error messages can contain source code line numbers.
The symbol table is also used by the debugger.

The #include lines are the C++ equvalent of a Go import directive.

The "C" import allows the Go code to refer to the C code, for example:

    driver C.RPDriver

creates a variable driver which is of C type RPDriver.
That's the C interface to the C++ Driver class.
Given this, a Go method can call a C++ method like so:

    func (d ConcreteRPlidarDriver) StartMotor() {
	C.RPDriverStartMotor(d.driver)
    }

For more details of Go calling C++ code via a C interface, read [the manual](https://golang.org/cmd/cgo/).  
The Go project github.com/burke/howto-go-with-cpp provides a simple worked example.

I haven't written any C or C++ software for over twenty years so my implementation
of the wrapper software may be more complicated than it needs to be.
