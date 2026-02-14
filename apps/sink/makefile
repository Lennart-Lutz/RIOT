# Set the name of your application:
APPLICATION = sink-app
 
# If no BOARD is found in the environment, use this default:
BOARD ?= nrf52840-mdk-dongle

UF2_SOFTDEV ?= DROP

# Use modules for networking
USEMODULE += gnrc
USEMODULE += netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_rpl
USEMODULE += gnrc_udp

# Peripherals
USEMODULE += ztimer
USEMODULE += ztimer_msec

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..
 
include $(RIOTBASE)/Makefile.include
