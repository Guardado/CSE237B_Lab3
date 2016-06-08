sudo iwpan dev wpan0 set pan_id 0xbeef       
sudo ip link add link wpan0 name lowpan0 type lowpan       
sudo ip -6 addr add 2001::1/64 dev lowpan0       
sudo ip link set wpan0 up       
sudo ip link set lowpan0 up       
