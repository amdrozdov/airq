# airq
Advanced Environment Analysis IoT

###Bootstrap the system
**Minimal**
* Turn on AIRq module
* Turn on AIRq gate
* Local IoT Mesh is ready, you can observe measurement on the screen

**MQTT**
* Install and run mosquitto with `apt-get install mosquitto`
* Setup local wifi ssid/pwd abd mosquito ip/port for IoT Gate
* Data is available in external network with MQTT topics

**Observation pipeline**
* Download and install Influx Telegraf
* Run influxdb `docker run --env-file=./env.sh --net=host -p 8086:8086 bitnami/influxdb:latest`
For `env.sh` you can put additional configuration, for example
```
INFLUXDB_HTTP_AUTH_ENABLED=false
INFLUXDB_ADMIN_USER_PASSWORD=test
INFLUXDB_ADMIN_USER_TOKEN=test
```
* Goto admin panel on `http://0.0.0.0:8086/`
* Create `airq` bucket
* Configure Telegraf settings in the admin panel for all measurements
```
[[inputs.mqtt_consumer]]
  servers = ["tcp://0.0.0.0:1883"]
  topics = [                                                     
    "/airq/dust/aqi",
    "/airq/dust/temperature",
    "/airq/dust/pressure",
    "/airq/dust/humidity",
    "/airq/dust/voc",
    "/airq/dust/pm1",
    "/airq/dust/pm2",
    "/airq/dust/pm10",
  ]
  topic_tag="topic"
  connection_timeout = "30s"
  data_format = "value"
  data_type = "float"
``` 
* Run telegraf using generated command and token
* IoT + MQTT Broker + DB export is ready

**Monitoring and Grafana dashboard**
* Start grafana docker with `docker run --net=host -p 3000:3000 grafana/grafana`
* Configure InfluxDB data source with IP of influxdb
* Load dashboard from the repo `grafana/dashboard.json`
* Done.
