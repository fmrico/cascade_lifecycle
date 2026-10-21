^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_cascade_lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.7 (2026-10-21)
------------------
* One version for Humble, Jazzy, Kilted and Lyrical. On Humble (from 1.1.0) the API does not change, but the class layout does: rebuild what was built against 1.1.0
* Do not abort when the state timer runs after shutdown, and do not ask for the node names then (`#25 <https://github.com/fmrico/cascade_lifecycle/issues/25>`_, `#27 <https://github.com/fmrico/cascade_lifecycle/issues/27>`_)
* Tests discover only nodes on this host (`#23 <https://github.com/fmrico/cascade_lifecycle/issues/23>`_)
* Tests with the SingleThreadedExecutor, 2 s per step: the experimental EventsExecutor is deprecated and Humble does not have it (`#28 <https://github.com/fmrico/cascade_lifecycle/issues/28>`_)
* Contributors: Francisco Martín Rico

2.0.6 (2026-07-28)
------------------
* Merge pull request `#21 <https://github.com/fmrico/cascade_lifecycle/issues/21>`_ from ahcorde/ahcorde/rolling/fix_dependency
  Fixed rclpy_cascade_lifecycle package.xml
* updated CMakeLists.txt
* Contributors: Alejandro Hernandez Cordero, Francisco Martín Rico

2.0.4 (2025-06-06)
------------------
* Fix deprecation of ament_target_dependencies
* Contributors: Francisco Martín Ricoo, David Lu, Alejandro Hernández Cordero

2.0.3 (2025-06-06)
------------------
* Change to EventsExecutor
* Explicit removal of node from executor before destroy in no dups
* Contributors: Francisco Martín Rico, David Lu, Alejandro Hernández Cordero

2.0.2 (2024-10-12)
------------------
* Explicit removal of node from executor before destroy
* Allow to have pubs/sub with standard QoS
* Contributors: Francisco Martín Rico, Juan Carlos Manzanares Serrano

2.0.0 (2024-04-04)
------------------
* Fix clear_activation
* Contributors: Francisco Martín Rico

1.0.2 (2022-12-31)
------------------
* Fix new deactivation publishers policy
* Contributors: Francisco Martín Rico

1.0.1 (2022-05-26)
------------------

1.0.0 (2021-07-03)
------------------
* Add support for namespace&Fix CI
  1. add support for namespace & test code
  2. fix CI config to v0.2
* Dynamic add/remove deps
* Contributors: Francisco Martin Rico, Francisco Martín Rico, Homalozoa

0.0.5 (2020-03-26)
------------------
* Fix shared_ptr ambiguous error
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.4 (2020-03-23)
------------------

0.0.3 (2020-03-22)
------------------
* Installing headers and libs
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Add license
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.2 (2020-03-13)
------------------
* Adding CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Complete functional version
* Nodes chained
* Contributors: Francisco Martin Rico
