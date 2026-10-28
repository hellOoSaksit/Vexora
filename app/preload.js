// The only bridge between the page and the main process: send commands, receive messages.
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('vexora', {
  send: (cmd) => ipcRenderer.send('sim:send', cmd),
  onMessage: (callback) => ipcRenderer.on('sim:message', (_event, msg) => callback(msg)),
});
