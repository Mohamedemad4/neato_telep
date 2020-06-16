import os
import sys
import argparse
import requests as req

parser = argparse.ArgumentParser(description='Push new OpenVPN config on the Neatos via mini_rest.py APIs')

parser.add_argument('--new_config',help='new OpenVPN config file')
parser.add_argument('--neato_ip_range',help='The IP range for the neatos (must be Nmap compatible)',default="192.168.1.1/25")
parser.add_argument('--mini_rest_port',help='the port for the mini_rest servers',default=7601)
args = parser.parse_args()

if args.new_config==None:
    print("Must supply --new_config /path/to/new_conf.ovpn")
    sys.exit(1)

def scan_for_neatos(port=7061,scan_range="192.168.1.1/25"):
    a=os.popen("nmap -p {0} {1}".format(port,scan_range)).read() # TODO: use subprocess and check it it exits cleanly    
    open_hosts=[]
    for line_idx,line in enumerate(a.split("\n")):
        if line.startswith(str(port)) and line.split(" ")[1]=="open":
            possible_neato=a.split("\n")[line_idx-4].replace("(","").replace(")","").split(" ")[-1]
            try:
                verify_neato=req.get("http://{0}:{1}/make_me_coffee".format(possible_neato,port))
                if verify_neato.__dict__["status_code"]==418:
                    open_hosts.append(possible_neato)
                    print("found host "+possible_neato)
            except Exception as e:
                log.warn(str(e))
    return open_hosts

new_open_conf=open(args.new_config,"r").read()

for ip in scan_for_neatos(port=args.mini_rest_port,scan_range=args.neato_ip_range):
    print("got a "+req.post("http://{0}:{1}/set_and_start_vpn_config".format(ip,args.mini_rest_port),data={"new_openvpn":new_open_conf}).content+" from Host "+ip)

