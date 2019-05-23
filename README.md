# cfsd-microservice-lynx_perception
ToBeDone
This is a microservice template for Chalmers Formula Student Driverless 19.

Run the image:
```
docker run --rm --net=host chalmersfsd/cfsd-template:v0.0.0 --cid=131 --verbose
```
Or
```
docker-compose up
```

Build the image:
```
docker build -t chalmersfsd/cfsd-template:v0.0.0 -f Dockerfile.amd64 .
```

## Features:
### sevices
There are two very basic message exchange service in the template:
1. a time triggered message sender running in 2Hz. 
2. two data triggered message receivers.

To add your logic and services you need to modify the Dockerfile.amd64 and CMakeLists.txt

### Message sets
It includes two standard message sets:
1. opendlv-standard-message-set
2. cfsd-extended-message-set
    * If cfsd-extended-message-set is not needed in the program, please remove the file along with the releated lines in cmake file.

### Travis:
To enabel travis, remove `secure` settings in travis.yml file and then use `travis encrypt` to generate new secure keys

Branches named by `test*` and tags will be automatically uploaded to our Docker hub

## Some Tools:

To exam the opendlv standard message reading:
```
docker run --rm --net=host -p 8080:8080 chalmersreveropendlv-signal-viewer-multi:v0.0.8 --cid=111
```

To exam the cfsd-extended-message reading:
```
docker run --rm -ti --init --net=host -v $PWD:/opchrberger/cluon-livefeed-multi:v0.0.121 --cid=111 --odvd=/opt/src/cfsd-extended-message-set-v0.0.1.odvd
```

To send the messages in opendlv-standard-message-set:
```
docker run --rm -ti --net=host wangroger0801/cfsd-message-controller:v0.0.3 --cid=111
```
