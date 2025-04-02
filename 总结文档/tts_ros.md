开启动作服务

```
ros2 launch tts_bringup tts.launch.py device:="cuda"
```

```
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World, How are you? Can you hear me? What is your favorite color? Do you know the Robot Operating System?'}"
```

中英文切换

![image-20250327132541226](https://nack-1316646329.cos.ap-nanjing.myqcloud.com/image-20250327132541226.png)

参数设置

```
chunk: Size of audio chunks to be sent to the audio player.
frame_id: Frame of for the tts.
model: The tts model. You can check the available models with tts --list_models.
model_path: Path to a local model file.
config_path: Path to a config file.
vocoder_path: Path to a vocoder model file.
vocoder_config_path: Path to a config file.
device: The device to run the model same as in torch.
speaker_wav: The wav file to perform voice cloning.
speaker: Which speaker voice to use for multi-speaker models. Check with tts --model_name <model> --list_language_idx.
stream: Whether to stream the audio data.
```

