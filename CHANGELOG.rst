demo_2019_02_25 (2019-02-09)
-----------------

New
~~~
- New Delay statistics message and online statistics algorithms
  calculations. [Jimi Damon]

  - allows mean and variance to be calculated incrementally when new data points come in
  - we can keep a sliding window of number of points to use for our mean and var calculations
  - resolved merge conflicts with taos checkin

  TMP
- Add CAN socket. [Tao Wang]

Other
~~~~~
- New topics for dji_sdk gimbal. [Unknown]
- Correct dji_sdk topics. [dan hennage]
- More vision topic fixes. [dan hennage]
- Fixed the naming bug in sensor camera topics. [dan hennage]
- Successfull build. [ubuntu]
- More dji_vision related topics and services. [ubuntu]
- Topics for dji vision. [ubuntu]
- Topic for Battery_state. [ubuntu]
- Distance/agl_lw. [dan hennage]
- Added ouster topics. [dan hennage]
- Add LED blink topic. [ubuntu]
- Added topics. [dan hennage]
- Set addr.can_family before ioctl. [Jimi Damon]
- Release CAN key when read fail. [dan hennage]
- :sparkles: Fixes for creating an installable library. [Jimi Damon]
- More user friendly message that includes stddev instead of variance.
  [Jimi Damon]
- Revise the header macro for CAN socket. [Tao Wang]
- Added topics. [dan hennage]
- Latency_wrapper.h bug fix. [ubuntu]
- Fix on latency_wrapper. [ubuntu]
- Added sc1 topics. [dan hennage]
- Changed to strings. [dan hennage]


