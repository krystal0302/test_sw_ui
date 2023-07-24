FROM node:16
RUN npm i -g pm2
RUN mkdir -p /home/farobot/far_app_data

COPY react_ui/ /app/react_ui
COPY server/ /app/server

# transpile -> server/public
RUN cd /app/react_ui && npm ci && npm run deploy
# NOTE: package cavas -> @mapbox/node-pre-gyp -> depends on ubuntu
RUN cd /app/server && npm install

WORKDIR /app/server
EXPOSE 3000
CMD ["pm2-dev", "start", "ecosystem.config.js"]

# example run:
# docker run -p 3000:3000 -v /host/machine/far_swarm_ui:/home/farobot/far_app_data far_swarm_ui:latest