### Building
1. Source your west python env
2. Navigate to your zephyr install
3. ```west build -p always -b open_rocket_tracker $project_dir -DBOARD_ROOT=$project_dir```
4. ```west flash```