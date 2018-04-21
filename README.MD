# Dead Reckoning Java Library

This library implements dead-reckoning algorithms defined by the Distributed Interactive Simulation (DIS) standard IEEE 
1278.1-1995.

Dead-reckoning is a process to calculate the position of an entity forward in time based on a previously known state 
(position, velocity, acceleration, etc.). This is also considered "extrapolation". This is particularly useful for 
distributed interactive simulations or networked video games where entity states need to be updated at high rates in 
clients (e.g. 60Hz).

# Dependencies

* Java8+
* apache commons-math3

# Build

This project is a maven project and can either be built using `mvn` command line tool or a Java IDE such as IntelliJ.

# Use

Include the library as a dependency in your project's `pom.xml`:
```xml
<dependencies>
    <dependency>
        <groupId>org.hmorgan</groupId>
        <artifactId>dead-reckoning</artifactId>
        <version>0.1</version>
    </dependency>
</dependencies>
```

# Examples

Most of the `DeadReckoningAlgorithm` implementations start with `DeadReckoningXXX`. Simply create an instance of the algorithm class
and begin updating the kinematic state with a new `EntityState`. 