# MuJoCo Humanoid Media Assets

This directory contains media files for marketplace presentation and documentation.

## Required Files

These files are needed for marketplace listing (managed by Neuraville):

- `thumbnail.png` (400x300px) - Marketplace card thumbnail
- `hero.png` (1200x600px) - Detail page hero banner
- `demo.mp4` (30-60 seconds) - Full demonstration video
- `demo.gif` (5-10 seconds, looping) - Quick preview animation
- `screenshots/` - Additional screenshots showcasing features

## Placeholder Status

🚧 **Media assets are pending creation**

To complete marketplace readiness, the following needs to be recorded:

### Demo Video Content (30-60 seconds)

1. **Opening** (5s): MuJoCo viewer with humanoid standing
2. **Walking** (15s): Humanoid walking forward, showing coordination
3. **Turning** (10s): Humanoid turning left/right
4. **Camera View** (10s): First-person view from humanoid's head camera
5. **Brain Activity** (10s): Brain Visualizer showing neural activity
6. **Connection** (5s): Terminal showing successful FEAGI connection

### Screenshots Needed

1. **Full Setup**: MuJoCo viewer + Brain Visualizer side-by-side
2. **Humanoid Close-up**: High-quality render of the humanoid model
3. **Walking Sequence**: 3-4 frame sequence showing gait
4. **Sensor Visualization**: Pressure sensors, camera view, gyro data

### Thumbnail Design

- Clear image of humanoid in action pose
- FEAGI branding
- Text: "MuJoCo Humanoid - 21 DOF Simulator"
- Professional, eye-catching design

### Hero Banner Design

- Wide shot of humanoid in MuJoCo environment
- Overlay: Key capabilities (vision, gyro, 21 servos, touch)
- Modern, clean design matching FEAGI brand

## Recording Setup

### Tools Recommended

- **Screen Recording**: OBS Studio (free, cross-platform)
- **Video Editing**: DaVinci Resolve (free) or Adobe Premiere
- **GIF Creation**: FFmpeg or online converter
- **Graphics**: Figma (free) or Adobe Photoshop

### Recording Instructions

1. **Start FEAGI Core**:
   ```bash
   docker compose -f playground.yml up
   ```

2. **Load Walking Genome** (once available):
   ```bash
   # Via API or Brain Visualizer
   ```

3. **Start Recording**:
   - Set resolution: 1920x1080 (1080p)
   - Frame rate: 30 FPS
   - Audio: Optional (add music or narration)

4. **Run MuJoCo Agent**:
   ```bash
   python controller.py --port 30000
   ```

5. **Capture**:
   - Full screen or application window
   - Show MuJoCo viewer prominently
   - Switch to Brain Visualizer periodically
   - Highlight active cortical areas

6. **Edit**:
   - Trim to 30-60 seconds
   - Add title card at start
   - Add end card with links
   - Export: MP4, H.264 codec, 1080p

7. **Create GIF**:
   ```bash
   ffmpeg -i demo.mp4 -vf "fps=10,scale=800:-1:flags=lanczos" \
     -t 10 -loop 0 demo.gif
   ```

8. **Screenshots**:
   - Use high resolution (1920x1080 or higher)
   - PNG format for quality
   - Clean composition (no clutter)

## File Specifications

### thumbnail.png
- **Dimensions**: 400x300px
- **Format**: PNG with transparency or JPEG
- **File Size**: < 200 KB
- **DPI**: 72 (web)

### hero.png
- **Dimensions**: 1200x600px
- **Format**: PNG or JPEG
- **File Size**: < 500 KB
- **DPI**: 72 (web)

### demo.mp4
- **Resolution**: 1920x1080 (1080p)
- **Duration**: 30-60 seconds
- **Format**: MP4 (H.264 codec)
- **Frame Rate**: 30 FPS
- **File Size**: < 50 MB
- **Audio**: Optional (AAC codec if present)

### demo.gif
- **Dimensions**: 800x600px (or similar)
- **Duration**: 5-10 seconds, looping
- **Format**: GIF
- **File Size**: < 5 MB
- **Frame Rate**: 10-15 FPS

### screenshots/*.png
- **Dimensions**: 1920x1080 or higher
- **Format**: PNG
- **File Size**: < 1 MB each
- **Quantity**: 3-4 images

## Marketplace Hosting

Once created, media files will be:
1. Reviewed by Neuraville
2. Uploaded to CDN (`cdn.feagi.io`)
3. Referenced in marketplace manifest
4. Displayed in FEAGI Desktop and Web marketplace

## Questions?

Contact Neuraville team for:
- Professional video production assistance
- Graphic design support
- Media hosting setup

---

**Status**: 🟡 Awaiting media creation  
**Last Updated**: 2025-11-18

