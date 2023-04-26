//const signalingUrl = 'wss://ayame-labo.shiguredo.app/signaling';
//const signalingUrl = 'ws://192.168.68.134:3000/signaling';
//const signalingUrl = 'ws://45.32.51.11:3000/signaling';
const signalingUrl = 'ws://35.77.141.241:3000/signaling';
let roomId = 'hoshianaaa@test1';
let clientId = null;
let videoCodec = null;
let audioCodec = null;
let signalingKey = "GrEh4Woqsm3d9G3Fd_eO3DBKd_SuWZheutMzdvrK3P00qzLB";

function onChangeVideoCodec() {
  videoCodec = document.getElementById("video-codec").value;
  if (videoCodec == 'none') {
    videoCodec = null;
  }
}
// query string から roomId, clientId を取得するヘルパー
function parseQueryString() {
  const qs = window.Qs;
  if (window.location.search.length > 0) {
    var params = qs.parse(window.location.search.substr(1));
    if (params.roomId) {
      roomId = params.roomId;
    }
    if (params.clientId) {
      clientId = params.clientId;
    }
    if (params.signalingKey) {
      signalingKey = params.signalingKey;
    }
  }
}


parseQueryString();

const roomIdInput = document.getElementById("roomIdInput");
roomIdInput.addEventListener('change', (event) => {
  console.log(event);
  roomId = event.target.value;
});