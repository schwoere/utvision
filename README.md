utvision
========
This is the utvision Ubitrack submodule.

Description
----------
The utvision contains algorithms working on (mostly) camera images.


Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the utvision by executing:

    git submodule add https://github.com/schwoere/utvision.git modules/utvision


Dependencies
----------
In addition, this module has to following submodule dependencies which have to be added for successful building:

<table>

  <tr>
    <th>Dependency</th><th>Dependent Components</th><th>optional Dependency</th>
  </tr>
  <tr>
    <td>utcore</td><td>utVision</td><td>no</td>
  </tr>
</table>
