# kubernetes deploy

Scripts for kubernetes deploy of all-in-one version of CSS (control service server). 


## Config files

### multus_config.yaml

Contains settings of ros-network - a multicast-enabled virtual network, which will be used for deployment of our services / pods. It is the same file as was in BED workshop.

The NIC and ip ranges have to be altered, based on the machine, where the kubernetes cluster runs. 

Use following commands to get all information needed:
```bash
ip route
```

The output should be similar to this:

```bash
default via 192.168.206.1 dev enp30s0 proto dhcp metric 100 
10.1.149.152 dev calibafe1e6cbef scope link 
10.1.149.153 dev cali6872062a950 scope link 
169.254.0.0/16 dev enp30s0 scope link metric 1000 
172.17.0.0/16 dev docker0 proto kernel scope link src 172.17.0.1 
192.168.206.0/24 dev enp30s0 proto kernel scope link src 192.168.206.196 metric 100
```

The first row, starting with default, is usually the interface, used for internet connection. The ip address after "default via" is the "gateway" in the multus_config.yaml, and the "enp30s0" after dev is the name of the default NIC, which should be filled in the "master" field, in the multus_config. The subnet is in this case "192.168.206.0/24" (just find the one, which has the same begin as the IP address on the first line) and the rangeStart and rangeEnd should be a range of IP addresses from the same subnet (be carefull to select available and large enough range).  
 
To apply the multus settings, following command should be used:

```bash
kubectl apply -f multus_config.yaml
```


### 5gera_css.yaml

Contains definition of one kubernetes pod, which will run the but5gera/css:latest docker image with the CSS (control service server). It has the hostNetwork enabled by default, which means any the ROS2 interface of the CSS is available on the host system. The CSS is deployed using:

```bash
kubectl apply -f 5gera_css.yaml
```

