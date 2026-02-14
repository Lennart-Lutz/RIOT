# Set the name of your application:
APPLICATION = plant-hub-app
 
# If no BOARD is found in the environment, use this default:
BOARD ?= plant-hub-v10-F411-233

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

USEMODULE += periph_adc
USEMODULE += periph_rtc
USEMODULE += periph_gpio
USEMODULE += periph_gpio_irq

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..
 
include $(RIOTBASE)/Makefile.include
