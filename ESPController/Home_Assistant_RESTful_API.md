# diyBMS v4
## Home Assistant RESTful API integration



### Secrets Configuration YAML file for Home Assistant
```
# Use this file to store secrets like usernames and passwords.
# Learn more at https://www.home-assistant.io/docs/configuration/secrets/

diybms_api_token: XXXXXXXXXXXXXXXXXXXXXXXX
```

### Example Configuration YAML file for Home Assistant
```
# Example configuration.yaml entry for Home Assistant integration with DIYBMS

rest:
  - resource: http://192.168.0.70/ha
    scan_interval: 10
    timeout: 5
    method: "GET"
    headers:
      Content-Type: application/json
      ApiKey: !secret diybms_api_token
    sensor:
      - unique_id: "diybms.activerules"
        value_template: "{{ value_json.activerules }}"
        name: "DIYBMS Active rules"
        state_class: "measurement"

      - unique_id: "diybms.chargemode"
        value_template: "{{ value_json.chgmode }}"
        name: "DIYBMS Charge mode"
        state_class: "measurement"

      - unique_id: "diybms.lowest_bank_voltage"
        value_template: "{{ value_json.lowbankv }}"
        name: "DIYBMS Lowest bank voltage"
        unit_of_measurement: "mV"
        device_class: "voltage"

      - unique_id: "diybms.highest_bank_voltage"
        value_template: "{{ value_json.highbankv }}"
        name: "DIYBMS Highest bank voltage"
        unit_of_measurement: "mV"
        device_class: "voltage"

      - unique_id: "diybms.lowest_cell_voltage"
        value_template: "{{ value_json.lowcellv }}"
        name: "DIYBMS Lowest cell voltage"
        unit_of_measurement: "mV"
        device_class: "voltage"

      - unique_id: "diybms.highest_cell_voltage"
        value_template: "{{ value_json.highcellv }}"
        name: "DIYBMS Highest cell voltage"
        unit_of_measurement: "mV"
        device_class: "voltage"

      - unique_id: "diybms.highest_external_temp"
        value_template: "{{ value_json.highextt }}"
        name: "DIYBMS Highest cell temperature"
        unit_of_measurement: "°C"
        device_class: "temperature"

      - unique_id: "diybms.highest_internal_temp"
        value_template: "{{ value_json.highintt }}"
        name: "DIYBMS Highest passive balance temperature"
        unit_of_measurement: "°C"
        device_class: "temperature"

      - unique_id: "diybms.current"
        value_template: "{% if 'c' in value_json %}{{ value_json.c }}{% else %}0{% endif %}"
        name: "DIYBMS DC Current"
        unit_of_measurement: "A"
        device_class: "current"
        icon: "mdi:current-dc"

      - unique_id: "diybms.voltage"
        value_template: "{% if 'v' in value_json %}{{ value_json.v }}{% else %}0{% endif %}"
        name: "DIYBMS DC voltage"
        unit_of_measurement: "V"
        device_class: "voltage"

      - unique_id: "diybms.power"
        value_template: "{% if 'pwr' in value_json %}{{ value_json.pwr }}{% else %}0{% endif %}"
        name: "DIYBMS Battery power"
        unit_of_measurement: "W"
        device_class: "power"

      - unique_id: "diybms.stateofcharge"
        value_template: "{% if 'soc' in value_json %}{{ value_json.soc }}{% else %}0{% endif %}"
        name: "DIYBMS State of charge"
        unit_of_measurement: "%"
        device_class: "battery"

      - unique_id: "diybms.dynamic_charge_voltage"
        value_template: "{% if 'dyncv' in value_json %}{{ value_json.dyncv }}{% else %}0{% endif %}"
        name: "DIYBMS Dynamic charge voltage"
        unit_of_measurement: "V"
        device_class: "voltage"

      - unique_id: "diybms.dynamic_charge_current"
        value_template: "{% if 'dyncc' in value_json %}{{ value_json.dyncc }}{% else %}0{% endif %}"
        name: "DIYBMS Dynamic charge current"
        unit_of_measurement: "A"
        device_class: "current"

    binary_sensor:
      - unique_id: "diybms.charge_allowed"
        value_template: "{{ value_json.chgallow }}"
        name: "DIYBMS Battery charging allowed"

      - unique_id: "diybms.discharge_allowed"
        value_template: "{{ value_json.dischgallow }}"
        name: "DIYBMS Battery discharging allowed"
```