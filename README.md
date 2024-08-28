# HOW TO USE?
## 1.Configure nginx

<div>

    server {
        listen 80;
        server_name 0.0.0.0;

        location / {
            root /home/dfg/auto_driver_demo/src/resource/html;
            index index.html;
        }
    }
</div>

## 2.TODO
