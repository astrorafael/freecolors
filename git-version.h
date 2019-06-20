/* From recipe given in
https://arduino.stackexchange.com/questions/23743/include-git-tag-or-svn-revision-in-arduino-sketch


Save the following file under the name make-git-version, somewhere in your PATH, and make it executable:

```bash
	#!/bin/bash

	# Go to the source directory.
	[ -n "$1" ] && cd "$1" || exit 1

	# Build a version string with git.
	version=$(git describe --tags --always --dirty 2> /dev/null)

	# If this is not a git repository, fallback to the compilation date.
	[ -n "$version" ] || version=$(date -I)

	# Save this in git-version.h.
	echo "#define GIT_VERSION \"$version\"" > $2/sketch/git-version.h
```
Locate the file named platform.txt in the Arduino installation directory 
(currently in arduino-1.8.5/hardware/arduino/avr for the AVR boards). 
In the same directory, create a file named platform.local.txt with the following content:

```
	hooks.sketch.prebuild.1.pattern=make-git-version "{build.source.path}" "{build.path}"
```

In your sketch, include "git-version.h" and use it like so:

```C
	#include "git-version.h"

	void setup() {
	  Serial.begin(9600);
	  Serial.println("This is version " GIT_VERSION
	}
```

Create an empty file in your sketch directory named git-version.h.
Note that the git-version.h in the current directory is only needed for the first compilation. 
The real git-version.h will be in the temporary build directory.

*/
