#!/usr/bin/env bash

# Snoop on UART TX/RX between dongle and the indoor unit
sigrok-cli -d fx2lafw -C D0,D1 -P uart:tx=D1:baudrate=9600:parity=even:tx_packet_delim=0xBB -P uart:rx=D0:baudrate=9600:parity=even:rx_packet_delim=0xBB -c samplerate="100 KHz" -O null -A uart=tx-packets:rx-packets --continuous | sed 's/uart-1:/TX: BB/;s/uart-2:/RX: BB/;s/ BB$//'