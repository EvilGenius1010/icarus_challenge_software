To build it, run
```
git clone https://github.com/EvilGenius1010/icarus_challenge_software.git
cd icarus_challenge_software
docker build -t challenge1 .
```

To run this, use 
```
docker run -it   --cap-add=sys_nice   --cap-add=ipc_lock   --cap-add=SYS_RESOURCE   --ulimit rtprio=99   --ulimit memlock=-1 --ulimit nice=-20 --sysctl fs.mqueue.msg_max=256 --sysctl fs.mqueue.msgsize_max=65536 --user root  --name adcs_sim   challenge1
```