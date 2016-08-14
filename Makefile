TARGETS = \
HeliSharpLib/bin/Debug/HeliSharp.dll \
HeliSharpTest/bin/Debug/HeliSharpTest.dll \
HeliSharpTool/bin/Debug/HeliSharpTool.exe

.PHONY: all test clean

all: $(TARGETS)

HeliSharpLib/bin/Debug/HeliSharp.dll: $(wildcard CsHeliLib/*.cs)
	mdtool build -p:HeliSharpLib

HeliSharpTest/bin/Debug/HeliSharpTest.dll: $(wildcard CsHeliLibTest/*.cs)
	mdtool build -p:HeliSharpTest

HeliSharpTool/bin/Debug/HeliSharpTool.exe: $(wildcard CsHeli/*.cs)
	mdtool build -p:HeliSharpTool

test: HeliSharpTest/bin/Debug/HeliSharpTest.dll
	nunit-console $^

clean:
	rm -rf */bin */obj

