// HoloLens C# Client — WebSocket JPEG Adaptation
// ==============================================
// Replace your existing aiortc-based video receiving code with this.
// Only the video-receiving part changes; head-rotation sending via
// DataChannel stays the same (still uses WebRTC for controls).
//
// If you want a pure-WebSocket solution (no WebRTC at all), see
// the "Pure WebSocket" section at the bottom.

// ================================================================
// Option A: Keep WebRTC for controls, add WebSocket for video
// ================================================================

using System;
using System.Net.WebSockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using System.IO;

public class WebSocketVideoReceiver : MonoBehaviour
{
    private ClientWebSocket _ws;
    private CancellationTokenSource _cts;
    private Texture2D _texture;

    public string serverUrl = "ws://YOUR_PC_IP:8765";
    public int videoWidth = 896;
    public int videoHeight = 504;

    async void Start()
    {
        _texture = new Texture2D(videoWidth, videoHeight, TextureFormat.RGB24, false);
        _cts = new CancellationTokenSource();
        await ConnectAndReceive();
    }

    async Task ConnectAndReceive()
    {
        _ws = new ClientWebSocket();
        await _ws.ConnectAsync(new Uri(serverUrl), _cts.Token);
        Debug.Log("WebSocket connected for video");

        var buffer = new byte[256 * 1024]; // 256 KB receive buffer
        var segment = new ArraySegment<byte>(buffer);

        while (_ws.State == WebSocketState.Open)
        {
            // Receive message
            var result = await _ws.ReceiveAsync(segment, _cts.Token);
            if (result.MessageType == WebSocketMessageType.Close)
                break;

            // Protocol: 4-byte length (big-endian) + JPEG bytes
            int totalLen = (int)result.Count;
            if (totalLen < 4) continue;

            int jpegLen = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
            int jpegStart = 4;

            // Handle fragmented messages (if JPEG is larger than buffer)
            byte[] jpegData;
            if (result.EndOfMessage)
            {
                jpegData = new byte[jpegLen];
                Array.Copy(buffer, jpegStart, jpegData, 0,
                    Math.Min(jpegLen, totalLen - jpegStart));
            }
            else
            {
                // Fragmented: collect all parts
                using (var ms = new MemoryStream())
                {
                    ms.Write(buffer, jpegStart, totalLen - jpegStart);
                    while (!result.EndOfMessage)
                    {
                        result = await _ws.ReceiveAsync(segment, _cts.Token);
                        ms.Write(buffer, 0, result.Count);
                    }
                    jpegData = ms.ToArray();
                }
            }

            // Decode JPEG and apply to texture
            if (_texture.LoadImage(jpegData))
            {
                // _texture now contains the decoded frame
                // Apply to your display material/renderer
            }
        }
    }

    void OnDestroy()
    {
        _cts?.Cancel();
        _ws?.Dispose();
    }

    // Your existing head-rotation sending code (via WebRTC DataChannel)
    // stays unchanged here.
}


// ================================================================
// Option B: Pure WebSocket (no WebRTC at all)
// ================================================================
// Video receive + head-rotation send, both via a single WebSocket.
// The server sends JPEG frames (4-byte length header + JPEG bytes).
// The client sends head rotation as 8-byte binary (same as DataChannel).

using System;
using System.Net.WebSockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class HoloLensWebSocketClient : MonoBehaviour
{
    private ClientWebSocket _ws;
    private CancellationTokenSource _cts;
    private Texture2D _texture;

    public string serverUrl = "ws://YOUR_PC_IP:8765";
    public int videoWidth = 896;
    public int videoHeight = 504;

    async void Start()
    {
        _texture = new Texture2D(videoWidth, videoHeight, TextureFormat.RGB24, false);
        _cts = new CancellationTokenSource();
        await ConnectAsync();
    }

    async Task ConnectAsync()
    {
        _ws = new ClientWebSocket();
        await _ws.ConnectAsync(new Uri(serverUrl), _cts.Token);
        Debug.Log("WebSocket connected");

        // Start receiving video
        var receiveTask = ReceiveVideoAsync();

        // Also send head rotation (if you have head tracking)
        while (_ws.State == WebSocketState.Open)
        {
            SendHeadRotation();
            await Task.Delay(33); // ~30 Hz
        }

        await receiveTask;
    }

    async Task ReceiveVideoAsync()
    {
        var buffer = new byte[256 * 1024];
        var segment = new ArraySegment<byte>(buffer);

        while (_ws.State == WebSocketState.Open)
        {
            var result = await _ws.ReceiveAsync(segment, _cts.Token);
            if (result.MessageType == WebSocketMessageType.Close)
                break;

            // Parse: 4-byte big-endian length + JPEG bytes
            int jpegLen = (buffer[0] << 24) | (buffer[1] << 16)
                        | (buffer[2] << 8) | buffer[3];
            byte[] jpegData = new byte[jpegLen];
            Array.Copy(buffer, 4, jpegData, 0, jpegLen);

            if (_texture.LoadImage(jpegData))
            {
                // Apply _texture to your display
            }
        }
    }

    async void SendHeadRotation()
    {
        // Get current head rotation from HoloLens
        float yaw = 0f;   // Replace with actual head tracking
        float pitch = 0f; // Replace with actual head tracking

        // Pack as 8 bytes: 2 x float32 little-endian
        byte[] data = new byte[8];
        BitConverter.GetBytes(yaw).CopyTo(data, 0);
        BitConverter.GetBytes(pitch).CopyTo(data, 4);

        try
        {
            await _ws.SendAsync(
                new ArraySegment<byte>(data),
                WebSocketMessageType.Binary,
                true,
                _cts.Token
            );
        }
        catch { }
    }

    void OnDestroy()
    {
        _cts?.Cancel();
        _ws?.Dispose();
    }
}
