^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package capabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2024-10-16)
------------------
* Fix linter error.
* Fix test for Python3.
* Replace yaml full_load with safe_load.
* Contributors: Patrick Chin

0.4.1 (2024-10-09)
------------------
* Fix regression bug, amend commit b017de3.
* Fix python3 bug.
* Remove obsolete imports.
* Contributors: Farhan Mustar, Patrick Chin

0.4.0 (2023-09-25)
------------------
* Mute yaml full load warning and fix missing basestring.
* Fix python3 error.
* Fix python shebangs.
* Contributors: Farhan Mustar, Patrick Chin

0.3.2 (2023-08-01)
------------------
* Fix code style.
* Add option to include stopping instances when retrieving running capabilities.
* Break the while loop if ROS is not OK.
* Add client API in C++.
* Fix tests.
* Switch from pep8 to pycodestyle and actually run it (`#93 <https://github.com/osrf/capabilities/issues/93>`_)
* Use itertools.chain to fix python3 test failure (`#92 <https://github.com/osrf/capabilities/issues/92>`_)

  * Use itertools.chain to fix python3 test failure.

* Support python3 by updating to package.xml format 3 (`#91 <https://github.com/osrf/capabilities/issues/91>`_)
* Bump CMake version to avoid CMP0048 warning (`#89 <https://github.com/osrf/capabilities/issues/89>`_)

  * Bump CMake version to avoid CMP0048 warning.
  * travisci: remove deprecated option for pip.
  * travisci: modernize things.
  * Change how coverage is invoked with rostest.

* Contributors: Patrick Chin, Scott K Logan, Shane Loretz, William Woodall, Wong Tze Lin

0.3.1 (2020-03-10)
------------------
* Updated ``package.xml`` to format 3 and and used condition dependencies to support Python3 (`#91 <https://github.com/osrf/capabilities/issues/91>`_)
* Contributors: William Woodall

0.3.0 (2020-03-09)
------------------
* Bump CMake version to avoid CMP0048 warning (`#89 <https://github.com/osrf/capabilities/issues/89>`_)
* Contributors: Shane Loretz

0.2.3 (2021-05-24)
------------------
* Fix import of coverage library.
* Contributors: Patrick Chin

0.2.2 (2020-07-19)
------------------
* Modify CapabilityServer to not exit on invalid default provider.
* Fix failing test.
* Contributors: Patrick Chin

0.2.1 (2019-02-17)
------------------
* Fix bug in get_providers service when default provider is not available.
* Modify behavior of free_capability.
  So that capability with running dependants are not stopped.
* Merge pull request `#84 <https://github.com/dfautomation/capabilities/issues/84>`_ from commaster90/fix-1
  Prune spec files by black/white lists
* Merge pull request `#83 <https://github.com/dfautomation/capabilities/issues/83>`_ from commaster90/master
  Move event subscribing ahead of service advertising. Fix `#82 <https://github.com/dfautomation/capabilities/issues/82>`_.
* Merge pull request `#80 <https://github.com/dfautomation/capabilities/issues/80>`_ from jonbinney/restart-fixes
  Fix typos in start_capability service
* Remove faulty start_capability test
  Can't rely on timing of calling stop_capability and then
  start_capability in quick succession, so this test can't be
  made to work reliably.
* Start capability returns an error code
* Update discovery unit test
* Add test for restarting capabilities
* Merge pull request `#76 <https://github.com/dfautomation/capabilities/issues/76>`_ from cottsay/master
  Export architecture_independent flag in package.xml
* Merge pull request `#79 <https://github.com/dfautomation/capabilities/issues/79>`_ from jonbinney/start-capability-response
  Return an error if start_capability requested for a capability that is already running
* Merge pull request `#77 <https://github.com/dfautomation/capabilities/issues/77>`_ from osrf/fix_tests
  debugging travis only failure
* Fix race condition in tests
* Contributors: Jon Binney, Patrick Chin, Scott K Logan, William Woodall

0.2.0 (2014-06-27)
------------------
* downgrade one of the exceptions to a warning
* fixup tests to reflect changes to client API
* Increase queue_size to 1000 for publishers
* Add queue_size arg for all publishers
* change exception behavior for use/free_capability in client API
* A rosdistro agnostic documentation reference
* conditionally try to stop reverse deps, since other reverse deps may have already stopped it
* make stopping the launch manager more robust to errors
* adds support of namespaces for capability nodelets
* Contributors: Jon Binney, Marcus Liebhardt, Nikolaus Demmel, William Woodall, kentsommer

0.1.1 (2014-05-02)
------------------
* Add entry in setup.py to install package data
* Fixed up testing
* Updates link to API doc
* Contributors: Marcus Liebhardt, William Woodall

0.1.0 (2014-04-15)
------------------
* First release
* Contributors: Esteve Fernandez, Marcus Liebhardt, Tully Foote, William Woodall
