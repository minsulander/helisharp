# HeliSharp - a C# library for helicopter flight simulation

Based on the original author's master thesis, originally a C++ project, adapted
to the new language and runtime platform, and extended from there.

## Project goals and style

Some objectives of this library:
- Provide an easy-to-use library for rotorcraft simulation
- Well tested with unit tests
- Contain tools for batch simulation and model validation
- Simple to integrate in a suitable runtime environment, i.e. [Unity3D](http://unity3d.com/), but 
  the library and tools themselves should be generic and not target a single specific runtime
- Clean and understandable code with minimal dependencies
- Code comments should reference scientific literature wherever that is relevant

The above goals have some implications on the development and coding style:
- Parameter variable names follow the Unity-style (public member variables) for parameters, for
  easy parametrization within the Unity Editor, but parametrization from a JSON file should also work
- Model states, inputs and outputs should be private variables or properties (so that they don't show up in the Unity Editor)
- Variable names sometimes use the obfuscated math notation (i.e. greek letters), like Omega_0, in
  order to maintain the ability to cross-reference the code with reference literature. Their declaration should be
  well commented. It's debatable whether this is a good thing... it's kind mutually exclusive with the
  simplicity of Unity Editor parametrization.

## Dependencies

The project should be able to compile with Microsoft Visual Studio, however that is not the original
developer's preferred platform, so a lot of the development has been done with MonoDevelop/Xamarin Studio
on Ubuntu Linux.

```bash
sudo apt install mono-complete nunit-console mono-reference-assemblies-2.0 octave
```

The project includes the following third-party DLLs:
- MathNet - some version (not the latest) that worked with Unity
- System.Threading - required by MathNet
- [SaladLab's light version of the Newtonsoft Json.NET library](https://github.com/SaladLab/Json.Net.Unity3D) 

## Building and running
Run the tests within an IDE or on the command line:
```bash
make test
```
To build validation graphs, run:
```bash
scripts/trimdata_compare.sh
scripts/simdata_compare.sh
```
and look at the results in the [validation/a109](validation/a109) folder.

## References and relevant literature
This is not a complete list of references but rather a reading list for those that are interested.
Links to Amazon where the original text is not available online.
 
- Insulander, Martin. MSc thesis. [Development of a Helicopter Simulator for Operator Interface Research](https://www.researchgate.net/profile/Martin_Insulander/publication/305723278_Development_of_a_Helicopter_Simulation_for_Operator_Interface_Research/links/579c7c0208ae80bf6ea47d03.pdf). 2008. The thesis for the original C++ project.
- Dreier, Mark. [Introduction to Helicopter and Tiltrotor Flight Simulation (Amazon)](https://www.amazon.com/Introduction-Helicopter-Tiltrotor-Simulation-Education/dp/1563478730).
- Padfield, Gareth D. [Helicopter Flight Dynamics: The Theory and Application of Flying Qualities and Simulation Modeling (Amazon)](https://www.amazon.com/Helicopter-Flight-Dynamics-AIAA-Education/dp/1563479206).
- Leishmann, J. Gordon. [Principles of Helicopter Aerodynamics (Amazon)](https://www.amazon.com/Principles-Helicopter-Aerodynamics-Cambridge-Aerospace/dp/0521858607).
- Heffley, Robert K. & Mnich, Marc A. [Minimum Complexity Helicopter Simulation Math Model](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19880020435.pdf). NASA CR 177476. 1988.
- Castles, Walter Jr & New, Noah C. [A Blade Element Analysis for Lifting Rotors that is Applicable for Large Inﬂow and Blade Angles and Reasonable Blade Geometry](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19930083300.pdf). NACA TN 2656. National Advisory Committee for Aeronautics, 1952.

Not about helicopter/flight simulation but interesting nonetheless:
- Nico Galoppo. [A presentation on rigid body dynamics simulation](http://www.cs.unc.edu/~lin/COMP768-F07/LEC/rbd1.pdf)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

