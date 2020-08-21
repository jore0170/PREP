This project is for running tx/rx in parallel
Both devices periodic advertise and set up a scan sync to exchange channel number and rssi values which are sent to a gatt characteristic to be received with matlab/html.  Right now the channel map is set to channels 0 and 1, CTE is not active in this project and gets the data from the scan response event.
