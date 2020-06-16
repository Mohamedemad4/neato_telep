#/usr/bin/bash
set -e

# install requests nmap flask rostools
# or maybe make me a dockerfile instead?

Problems:
    - IP conflict on duplicate-cn
add those to the /etc/openvpn/server/server.conf
local 192.168.1.3
port 1194
proto tcp
dev tun
ca ca.crt
cert server.crt
key server.key
dh dh.pem
auth SHA512
tls-crypt tc.key
topology subnet
client-to-client 
duplicate-cn 
server 10.8.0.0 255.255.255.0
push "redirect-gateway def1 bypass-dhcp"
ifconfig-pool-persist ipp.txt
push "dhcp-option DNS 8.8.8.8"
push "dhcp-option DNS 8.8.4.4"
keepalive 10 120
cipher AES-256-CBC
user nobody
group nogroup
persist-key
persist-tun
status openvpn-status.log
verb 3
crl-verify crl.pem