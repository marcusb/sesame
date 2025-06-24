echo 'syslog_config: { enabled: true, syslog_host: "nuc.example.org", syslog_port: 514 }' | protoc --encode=LoggingConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/cfg/logging'


echo 'enabled: true, broker_host: "mqtt.example.org", broker_port: 1883, client_id: "sesame-dev", username: "sesame", password: "***REMOVED***", prefix: "sesame-dev"' | protoc --encode=MqttConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/cfg/mqtt

echo 'hostname: "sesame", ssid: "MY_WIFI", security: 2, password: "seekrit"' | protoc --encode=NetworkConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://192.168.4.1/cfg/network'

echo 'url: "http://example.org/sesame.bin"'  | protoc --encode=FirmwareUpgradeFetchRequest proto/api.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/fwupgrade'
