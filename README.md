utvision
========
This is the utvision Ubitrack submodule.

Description
----------
The utvision contains algorithms working on (mostly) camera images.


Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the utvision by executing:

    git submodule add https://github.com/Ubitrack/utvision.git modules/utvision


Dependencies
----------
In addition, this module has to following submodule dependencies which have to be added for successful building:


<table>
  <tr>
    <th>Component</th><th>Dependency</th>
  </tr>
  <tr>
    <td>all</td><td>utCore</td>
  </tr>
</table>
