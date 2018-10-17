#!/bin/bash

echo "Launch this shell script before calling simple_udp_receiver"
echo "Then write something and that will be sent to simple_udp_receiver"

nc -l -u 1234 
