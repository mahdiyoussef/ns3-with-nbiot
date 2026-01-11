set(name nbiot)

set(source_files
    model/nbiot-common.h
    model/nbiot-phy.cc
    model/nbiot-ue-phy.cc
    model/nbiot-enb-phy.cc
    model/nbiot-control-messages.cc
    model/nbiot-mac.cc
    model/nbiot-ue-mac.cc
    model/nbiot-enb-mac.cc
    model/nbiot-sps-scheduler.cc
    model/nbiot-latency-tag.cc
    model/nbiot-rlc.cc
    model/nbiot-pdcp.cc
    model/nbiot-rrc.cc
    model/nbiot-power-saving.cc
    model/nbiot-net-device.cc
    helper/nbiot-helper.cc
)

set(header_files
    model/nbiot-common.h
    model/nbiot-phy.h
    model/nbiot-ue-phy.h
    model/nbiot-enb-phy.h
    model/nbiot-control-messages.h
    model/nbiot-mac.h
    model/nbiot-ue-mac.h
    model/nbiot-enb-mac.h
    model/nbiot-sps-scheduler.h
    model/nbiot-latency-tag.h
    model/nbiot-rlc.h
    model/nbiot-pdcp.h
    model/nbiot-rrc.h
    model/nbiot-power-saving.h
    model/nbiot-net-device.h
    helper/nbiot-helper.h
)

set(test_sources
    test/nbiot-test-suite.cc
)

set(libraries_to_link
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libspectrum}
    ${libpropagation}
    ${libmobility}
    ${libantenna}
    ${liblte}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
    TEST_SOURCES ${test_sources}
)

# Build examples
build_lib_example(
    NAME nbiot-standalone-example
    SOURCE_FILES examples/nbiot-standalone-example.cc
    LIBRARIES_TO_LINK ${libnbiot}
)

build_lib_example(
    NAME nbiot-sps-simulation
    SOURCE_FILES examples/nbiot-sps-simulation.cc
    LIBRARIES_TO_LINK ${libnbiot}
)

build_lib_example(
    NAME nbiot-sps-latency-simulation
    SOURCE_FILES examples/nbiot-sps-latency-simulation.cc
    LIBRARIES_TO_LINK ${libnbiot}
)

build_lib_example(
    NAME nbiot-simulation-article
    SOURCE_FILES examples/nbiot-simulation-article.cc
    LIBRARIES_TO_LINK ${libnbiot}
)
