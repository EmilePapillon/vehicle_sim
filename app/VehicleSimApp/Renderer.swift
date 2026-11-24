import AppKit
import MetalKit
import simd
import VehicleCAPI

// MARK: - Math

extension float4x4 {
  static func translation(_ t: SIMD3<Float>) -> float4x4 {
    var m = matrix_identity_float4x4
    m.columns.3 = SIMD4(t, 1)
    return m
  }
  static func scale(_ s: SIMD3<Float>) -> float4x4 { float4x4(diagonal: SIMD4(s, 1)) }
  static func rotation(_ angle: Float, _ axis: SIMD3<Float>) -> float4x4 {
    float4x4(simd_quatf(angle: angle, axis: normalize(axis)))
  }
  static func rotZ(_ a: Float) -> float4x4 { rotation(a, [0, 0, 1]) }
  static func rotY(_ a: Float) -> float4x4 { rotation(a, [0, 1, 0]) }
  static func rotX(_ a: Float) -> float4x4 { rotation(a, [1, 0, 0]) }

  /// Right-handed perspective projection with Metal's [0, 1] depth range.
  static func perspective(fovy: Float, aspect: Float, near: Float, far: Float) -> float4x4 {
    let ys = 1 / tan(fovy * 0.5), xs = ys / aspect, zs = far / (near - far)
    return float4x4(columns: (SIMD4(xs, 0, 0, 0), SIMD4(0, ys, 0, 0), SIMD4(0, 0, zs, -1), SIMD4(0, 0, zs * near, 0)))
  }

  static func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> float4x4 {
    let f = normalize(center - eye), s = normalize(cross(f, up)), u = cross(s, f)
    return float4x4(columns: (SIMD4(s.x, u.x, -f.x, 0), SIMD4(s.y, u.y, -f.y, 0), SIMD4(s.z, u.z, -f.z, 0),
                              SIMD4(-dot(s, eye), -dot(u, eye), dot(f, eye), 1)))
  }
}

// MARK: - GPU data

private struct Vertex {
  var position: SIMD3<Float>
  var normal: SIMD3<Float>
}

private struct Instance {
  var model: float4x4
  var color: SIMD4<Float>
}

private struct Uniforms {
  var viewProj: float4x4
  var lightDir: SIMD4<Float>
  var cameraPos: SIMD4<Float>
  var fogColor: SIMD4<Float>
}

private let shaderSource = """
#include <metal_stdlib>
using namespace metal;

struct VIn { float3 pos [[attribute(0)]]; float3 nrm [[attribute(1)]]; };
struct Inst { float4x4 model; float4 color; };
struct Uni { float4x4 viewProj; float4 lightDir; float4 camPos; float4 fog; };
struct VOut { float4 pos [[position]]; float3 nrm; float4 color; float3 world; };

vertex VOut mesh_vs(VIn in [[stage_in]], constant Uni& u [[buffer(1)]],
                    const device Inst* inst [[buffer(2)]], uint iid [[instance_id]]) {
  Inst I = inst[iid];
  float4 w = I.model * float4(in.pos, 1.0);
  VOut o;
  o.pos = u.viewProj * w;
  o.world = w.xyz;
  o.nrm = (I.model * float4(in.nrm, 0.0)).xyz;
  o.color = I.color;
  return o;
}

float3 applyFog(float3 c, float3 world, constant Uni& u) {
  float d = length(u.camPos.xyz - world);
  return mix(c, u.fog.rgb, clamp((d - 80.0) / 260.0, 0.0, 1.0));
}

fragment float4 lit_fs(VOut in [[stage_in]], constant Uni& u [[buffer(1)]]) {
  float3 n = normalize(in.nrm);
  float3 l = -u.lightDir.xyz;
  float diff = max(dot(n, l), 0.0);
  float3 v = normalize(u.camPos.xyz - in.world);
  float spec = pow(max(dot(n, normalize(v + l)), 0.0), 48.0) * 0.35;
  float sky = 0.5 + 0.5 * n.z;
  float3 c = in.color.rgb * (0.22 + 0.18 * sky + 0.7 * diff) + spec;
  return float4(applyFog(c, in.world, u), in.color.a);
}

fragment float4 unlit_fs(VOut in [[stage_in]], constant Uni& u [[buffer(1)]]) {
  return float4(applyFog(in.color.rgb, in.world, u), in.color.a);
}

float gridLine(float2 p, float width) {
  float2 g = abs(fract(p - 0.5) - 0.5) / (fwidth(p) * width);
  return 1.0 - min(min(g.x, g.y), 1.0);
}

fragment float4 ground_fs(VOut in [[stage_in]], constant Uni& u [[buffer(1)]]) {
  float2 p = in.world.xy;
  float3 c = float3(0.26, 0.28, 0.30);
  float2 cell = floor(p / 10.0);
  c += 0.015 * fmod(abs(cell.x + cell.y), 2.0);
  c = mix(c, float3(0.36, 0.38, 0.41), gridLine(p, 1.0) * 0.55);
  c = mix(c, float3(0.55, 0.58, 0.62), gridLine(p / 10.0, 1.5) * 0.8);
  return float4(applyFog(c, in.world, u), 1.0);
}
"""

private struct Mesh {
  let vertices: MTLBuffer
  let indices: MTLBuffer
  let count: Int

  init(device: MTLDevice, vertices v: [Vertex], indices i: [UInt16]) {
    vertices = device.makeBuffer(bytes: v, length: MemoryLayout<Vertex>.stride * v.count)!
    indices = device.makeBuffer(bytes: i, length: MemoryLayout<UInt16>.stride * i.count)!
    count = i.count
  }

  /// Unit cube centred on the origin.
  static func cube(device: MTLDevice) -> Mesh {
    var v: [Vertex] = []
    var idx: [UInt16] = []
    let faces: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)] = [
      ([1, 0, 0], [0, 1, 0], [0, 0, 1]), ([-1, 0, 0], [0, 0, 1], [0, 1, 0]),
      ([0, 1, 0], [0, 0, 1], [1, 0, 0]), ([0, -1, 0], [1, 0, 0], [0, 0, 1]),
      ([0, 0, 1], [1, 0, 0], [0, 1, 0]), ([0, 0, -1], [0, 1, 0], [1, 0, 0]),
    ]
    for (n, a, b) in faces {
      let base = UInt16(v.count)
      for (sa, sb) in [(-1, -1), (1, -1), (1, 1), (-1, 1)] as [(Float, Float)] {
        v.append(Vertex(position: 0.5 * (n + sa * a + sb * b), normal: n))
      }
      idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    }
    return Mesh(device: device, vertices: v, indices: idx)
  }

  /// Unit-radius cylinder along +Y with height 1, centred on the origin.
  static func cylinder(device: MTLDevice, segments: Int = 36) -> Mesh {
    var v: [Vertex] = []
    var idx: [UInt16] = []
    for i in 0...segments {
      let a = Float(i) / Float(segments) * 2 * .pi
      let n = SIMD3<Float>(cos(a), 0, sin(a))
      v.append(Vertex(position: [n.x, -0.5, n.z], normal: n))
      v.append(Vertex(position: [n.x, 0.5, n.z], normal: n))
    }
    for i in 0..<segments {
      let b = UInt16(2 * i)
      idx += [b, b + 2, b + 1, b + 1, b + 2, b + 3]
    }
    for side: Float in [-0.5, 0.5] {
      let centre = UInt16(v.count)
      let n = SIMD3<Float>(0, side * 2, 0)
      v.append(Vertex(position: [0, side, 0], normal: n))
      for i in 0...segments {
        let a = Float(i) / Float(segments) * 2 * .pi
        v.append(Vertex(position: [cos(a), side, sin(a)], normal: n))
      }
      for i in 0..<segments {
        let a = centre + 1 + UInt16(i)
        idx += side > 0 ? [centre, a, a + 1] : [centre, a + 1, a]
      }
    }
    return Mesh(device: device, vertices: v, indices: idx)
  }

  /// Unit quad in the XY plane facing +Z.
  static func quad(device: MTLDevice) -> Mesh {
    let n = SIMD3<Float>(0, 0, 1)
    let v = [Vertex(position: [-0.5, -0.5, 0], normal: n), Vertex(position: [0.5, -0.5, 0], normal: n),
             Vertex(position: [0.5, 0.5, 0], normal: n), Vertex(position: [-0.5, 0.5, 0], normal: n)]
    return Mesh(device: device, vertices: v, indices: [0, 1, 2, 0, 2, 3])
  }
}

// MARK: - Camera

enum CameraMode: String, CaseIterable, Identifiable {
  case chase = "Chase", orbit = "Orbit", top = "Top"
  var id: String { rawValue }
}

struct ViewOptions: Equatable {
  var camera: CameraMode = .chase
  var rollExaggeration: Double = 2.0
  var forceScale: Double = 1.0
  var showForces = true
  var showLoads = true
  var showTrail = true
}

// MARK: - Renderer

@MainActor
final class Renderer: NSObject, MTKViewDelegate {
  let device: MTLDevice
  private let queue: MTLCommandQueue
  private let litPipeline, unlitPipeline, groundPipeline: MTLRenderPipelineState
  private let depthOn, depthReadOnly, depthAlways: MTLDepthStencilState
  private let cube, cylinder, quad: Mesh
  static let sampleCount = 4
  static let colorFormat = MTLPixelFormat.bgra8Unorm_srgb
  static let depthFormat = MTLPixelFormat.depth32Float

  weak var engine: SimEngine?
  var options = ViewOptions()

  // Orbit camera state (mouse-driven).
  var orbitAzimuth: Float = 2.4
  var orbitElevation: Float = 0.35
  var distance: Float = 11
  private var chaseYaw: Float = 0
  private var lastFrame = CACurrentMediaTime()

  init?(device: MTLDevice) {
    self.device = device
    guard let q = device.makeCommandQueue() else { return nil }
    queue = q
    do {
      let lib = try device.makeLibrary(source: shaderSource, options: nil)
      let vd = MTLVertexDescriptor()
      vd.attributes[0].format = .float3
      vd.attributes[0].offset = 0
      vd.attributes[0].bufferIndex = 0
      vd.attributes[1].format = .float3
      vd.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
      vd.attributes[1].bufferIndex = 0
      vd.layouts[0].stride = MemoryLayout<Vertex>.stride
      func pipeline(_ fs: String, blend: Bool) throws -> MTLRenderPipelineState {
        let d = MTLRenderPipelineDescriptor()
        d.vertexFunction = lib.makeFunction(name: "mesh_vs")
        d.fragmentFunction = lib.makeFunction(name: fs)
        d.vertexDescriptor = vd
        d.colorAttachments[0].pixelFormat = Renderer.colorFormat
        d.depthAttachmentPixelFormat = Renderer.depthFormat
        d.rasterSampleCount = Renderer.sampleCount
        if blend {
          let c = d.colorAttachments[0]!
          c.isBlendingEnabled = true
          c.sourceRGBBlendFactor = .sourceAlpha
          c.destinationRGBBlendFactor = .oneMinusSourceAlpha
          c.sourceAlphaBlendFactor = .one
          c.destinationAlphaBlendFactor = .oneMinusSourceAlpha
        }
        return try device.makeRenderPipelineState(descriptor: d)
      }
      litPipeline = try pipeline("lit_fs", blend: false)
      unlitPipeline = try pipeline("unlit_fs", blend: true)
      groundPipeline = try pipeline("ground_fs", blend: false)
    } catch {
      NSLog("Renderer: shader compilation failed: \(error)")
      return nil
    }
    let ds = MTLDepthStencilDescriptor()
    ds.depthCompareFunction = .less
    ds.isDepthWriteEnabled = true
    depthOn = device.makeDepthStencilState(descriptor: ds)!
    ds.isDepthWriteEnabled = false
    ds.depthCompareFunction = .lessEqual
    depthReadOnly = device.makeDepthStencilState(descriptor: ds)!
    ds.depthCompareFunction = .always
    depthAlways = device.makeDepthStencilState(descriptor: ds)!
    cube = .cube(device: device)
    cylinder = .cylinder(device: device)
    quad = .quad(device: device)
    super.init()
  }

  nonisolated func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

  nonisolated func draw(in view: MTKView) {
    MainActor.assumeIsolated {
      guard let pass = view.currentRenderPassDescriptor, let drawable = view.currentDrawable,
            let cmd = queue.makeCommandBuffer() else { return }
      encode(cmd: cmd, pass: pass, size: view.drawableSize)
      cmd.present(drawable)
      cmd.commit()
    }
  }

  static let clearColor = MTLClearColor(red: 0.58, green: 0.68, blue: 0.78, alpha: 1)

  /// Renders one frame into an offscreen texture and returns it as PNG data.
  func renderPNG(width: Int, height: Int) -> Data? {
    let msDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: Renderer.colorFormat, width: width, height: height, mipmapped: false)
    msDesc.textureType = .type2DMultisample
    msDesc.sampleCount = Renderer.sampleCount
    msDesc.usage = .renderTarget
    msDesc.storageMode = .private
    let resolveDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: Renderer.colorFormat, width: width, height: height, mipmapped: false)
    resolveDesc.usage = [.renderTarget, .shaderRead]
    resolveDesc.storageMode = .shared
    let depthDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: Renderer.depthFormat, width: width, height: height, mipmapped: false)
    depthDesc.textureType = .type2DMultisample
    depthDesc.sampleCount = Renderer.sampleCount
    depthDesc.usage = .renderTarget
    depthDesc.storageMode = .private
    guard let ms = device.makeTexture(descriptor: msDesc), let resolve = device.makeTexture(descriptor: resolveDesc),
          let depth = device.makeTexture(descriptor: depthDesc), let cmd = queue.makeCommandBuffer() else { return nil }
    let pass = MTLRenderPassDescriptor()
    pass.colorAttachments[0].texture = ms
    pass.colorAttachments[0].resolveTexture = resolve
    pass.colorAttachments[0].loadAction = .clear
    pass.colorAttachments[0].storeAction = .multisampleResolve
    pass.colorAttachments[0].clearColor = Renderer.clearColor
    pass.depthAttachment.texture = depth
    pass.depthAttachment.loadAction = .clear
    pass.depthAttachment.storeAction = .dontCare
    pass.depthAttachment.clearDepth = 1
    encode(cmd: cmd, pass: pass, size: CGSize(width: width, height: height))
    cmd.commit()
    cmd.waitUntilCompleted()

    var bytes = [UInt8](repeating: 0, count: width * height * 4)
    resolve.getBytes(&bytes, bytesPerRow: width * 4, from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
    guard let provider = CGDataProvider(data: Data(bytes) as CFData),
          let image = CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: width * 4,
                              space: CGColorSpace(name: CGColorSpace.sRGB)!,
                              bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue),
                              provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    else { return nil }
    return NSBitmapImageRep(cgImage: image).representation(using: .png, properties: [:])
  }

  // MARK: Scene

  private func encode(cmd: MTLCommandBuffer, pass: MTLRenderPassDescriptor, size: CGSize) {
    guard let engine, let enc = cmd.makeRenderCommandEncoder(descriptor: pass) else { return }
    let now = CACurrentMediaTime()
    let frameDt = Float(min(now - lastFrame, 0.1))
    lastFrame = now

    let t = engine.telemetry
    let g = engine.geometry
    let carPos = SIMD3<Float>(Float(t.x), Float(t.y), 0)
    let yaw = Float(t.yaw)

    // Camera
    var d = yaw - chaseYaw
    while d > .pi { d -= 2 * .pi }
    while d < -.pi { d += 2 * .pi }
    chaseYaw += d * min(1, frameDt * 4)
    let target = carPos + SIMD3(0, 0, 0.7)
    let eye: SIMD3<Float>
    var up = SIMD3<Float>(0, 0, 1)
    switch options.camera {
    case .chase:
      eye = target + SIMD3(-cos(chaseYaw) * distance, -sin(chaseYaw) * distance, distance * 0.32)
    case .orbit:
      let ce = cos(orbitElevation)
      eye = target + distance * SIMD3(ce * cos(orbitAzimuth), ce * sin(orbitAzimuth), sin(orbitElevation))
    case .top:
      eye = target + SIMD3(0, 0, distance * 2.6)
      up = SIMD3(cos(yaw), sin(yaw), 0)
    }
    let aspect = Float(max(size.width, 1) / max(size.height, 1))
    let proj = float4x4.perspective(fovy: 0.85, aspect: aspect, near: 0.1, far: 600)
    let view = float4x4.lookAt(eye: eye, center: target, up: up)
    var uni = Uniforms(viewProj: proj * view, lightDir: SIMD4(normalize(SIMD3<Float>(-0.4, -0.3, -1)), 0),
                       cameraPos: SIMD4(eye, 1), fogColor: SIMD4(0.58, 0.68, 0.78, 1))

    enc.setVertexBytes(&uni, length: MemoryLayout<Uniforms>.stride, index: 1)
    enc.setFragmentBytes(&uni, length: MemoryLayout<Uniforms>.stride, index: 1)

    // Ground: a large quad that follows the car (grid is computed in world space).
    enc.setRenderPipelineState(groundPipeline)
    enc.setDepthStencilState(depthOn)
    draw(enc, quad, [Instance(model: .translation(SIMD3(round(carPos.x), round(carPos.y), 0)) * .scale([1200, 1200, 1]),
                              color: [1, 1, 1, 1])])

    // Lines: reference path and driven trail, as thin flat ribbons.
    enc.setRenderPipelineState(unlitPipeline)
    enc.setDepthStencilState(depthReadOnly)
    if engine.referencePath.count > 1 {
      ribbon(enc, engine.referencePath, width: 0.12, z: 0.006, color: [0.98, 0.82, 0.25, 0.9])
    }
    if options.showTrail, engine.trail.count > 1 {
      ribbon(enc, engine.trail + [SIMD2(t.x, t.y)], width: 0.08, z: 0.008, color: [0.35, 0.85, 1.0, 0.85])
    }
    // Blob shadow
    let carRot = float4x4.translation(carPos) * .rotZ(yaw)
    let bodyLen = Float(g.wheelbase) + 1.75, bodyWid = Float(min(g.track_front, g.track_rear)) - 0.12
    let bodyCx = Float(g.a - g.b) * 0.5
    draw(enc, quad, [Instance(model: carRot * .translation([bodyCx, 0, 0.01]) * .scale([bodyLen + 0.4, bodyWid + 0.7, 1]),
                              color: [0, 0, 0, 0.28])])

    // Solid geometry
    enc.setRenderPipelineState(litPipeline)
    enc.setDepthStencilState(depthOn)
    var boxes: [Instance] = []
    var cyls: [Instance] = []
    var overlay: [Instance] = []  // force vectors, drawn on top of the car

    let k = Float(options.rollExaggeration)
    let cgH = Float(g.cg_height)
    let body = carRot * .translation([0, 0, cgH + Float(t.heave) * k]) * .rotY(Float(t.pitch) * k) *
      .rotX(Float(t.roll) * k) * .translation([0, 0, -cgH])
    let paint = SIMD4<Float>(0.80, 0.16, 0.12, 1)
    let rf = Float(g.wheel_radius_front), rr = Float(g.wheel_radius_rear)
    // Chassis slab between the wheels, upper body above the wheels, cabin on top.
    boxes.append(Instance(model: body * .translation([bodyCx, 0, 0.42]) * .scale([bodyLen, bodyWid, 0.36]), color: paint))
    boxes.append(Instance(model: body * .translation([bodyCx, 0, 2 * rf + 0.12]) * .scale([bodyLen, bodyWid + 0.42, 0.2]), color: paint))
    boxes.append(Instance(model: body * .translation([bodyCx - 0.3, 0, 2 * rf + 0.45]) * .scale([bodyLen * 0.48, bodyWid * 0.9, 0.46]),
                          color: [0.16, 0.2, 0.26, 1]))
    boxes.append(Instance(model: body * .translation([bodyCx + bodyLen * 0.5, 0, 2 * rf + 0.12]) * .scale([0.04, bodyWid, 0.1]),
                          color: [0.95, 0.95, 0.85, 1]))  // headlight strip marks the front

    let wheels = t.wheels
    let pos: [SIMD2<Float>] = [
      [Float(g.a), Float(g.track_front) / 2], [Float(g.a), -Float(g.track_front) / 2],
      [-Float(g.b), Float(g.track_rear) / 2], [-Float(g.b), -Float(g.track_rear) / 2],
    ]
    let (staticF, staticR) = engine.staticLoads
    for i in 0..<4 {
      let w = wheels[i]
      let r = i < 2 ? rf : rr
      let p = pos[i]
      let hub = carRot * .translation([p.x, p.y, r]) * .rotZ(Float(w.steer))
      let spin = hub * .rotY(Float(w.spin_angle))
      cyls.append(Instance(model: spin * .scale([r, 0.22, r]), color: [0.13, 0.13, 0.14, 1]))
      cyls.append(Instance(model: spin * .translation([0, i % 2 == 0 ? 0.115 : -0.115, 0]) * .scale([r * 0.62, 0.02, r * 0.62]),
                           color: [0.65, 0.67, 0.7, 1]))
      boxes.append(Instance(model: spin * .translation([0, i % 2 == 0 ? 0.13 : -0.13, r * 0.35]) * .scale([0.07, 0.02, r * 0.6]),
                            color: [0.9, 0.75, 0.2, 1]))  // spoke marker shows rotation

      let contact = carRot * SIMD4<Float>(p.x, p.y, 0.05, 1)
      let c3 = SIMD3(contact.x, contact.y, contact.z)
      let heading = yaw + Float(w.steer)
      let s = Float(options.forceScale) / 3000  // metres per newton
      if options.showForces {
        arrow(&overlay, from: c3, angle: heading, length: Float(w.fx) * s, color: [0.25, 0.85, 0.35, 0.95])
        arrow(&overlay, from: c3, angle: heading + .pi / 2, length: Float(w.fy) * s, color: [0.95, 0.3, 0.25, 0.95])
      }
      if options.showLoads {
        let ratio = Float(w.fz / (i < 2 ? staticF : staticR))
        let h = max(Float(w.fz) / 4000 * 0.9, 0.005)
        let col = mix(SIMD4<Float>(0.25, 0.55, 1.0, 0.9), SIMD4<Float>(1.0, 0.45, 0.15, 0.9), t: simd_clamp((ratio - 0.4) / 1.2, 0, 1))
        let side: Float = i % 2 == 0 ? 0.32 : -0.32
        boxes.append(Instance(model: carRot * .translation([p.x, p.y + side, 2 * r + 0.25 + h / 2]) * .scale([0.12, 0.12, h]), color: col))
      }
    }
    draw(enc, cube, boxes)
    draw(enc, cylinder, cyls)
    enc.setRenderPipelineState(unlitPipeline)
    enc.setDepthStencilState(depthAlways)
    draw(enc, cube, overlay)
    enc.endEncoding()
  }

  private func arrow(_ out: inout [Instance], from: SIMD3<Float>, angle: Float, length: Float, color: SIMD4<Float>) {
    guard abs(length) > 0.02 else { return }
    let a = length >= 0 ? angle : angle + .pi
    let len = abs(length)
    out.append(Instance(model: .translation(from) * .rotZ(a) * .translation([len / 2, 0, 0]) * .scale([len, 0.06, 0.06]), color: color))
    out.append(Instance(model: .translation(from) * .rotZ(a) * .translation([len, 0, 0]) * .rotZ(.pi / 4) * .scale([0.16, 0.16, 0.08]),
                        color: color))
  }

  private func ribbon(_ enc: MTLRenderCommandEncoder, _ pts: [SIMD2<Double>], width: Float, z: Float, color: SIMD4<Float>) {
    // Segments as flat quads (Metal has no wide lines). Decimate long paths.
    let stride = max(1, pts.count / 1500)
    var inst: [Instance] = []
    var i = 0
    while i + stride < pts.count {
      let a = SIMD2<Float>(Float(pts[i].x), Float(pts[i].y)), b = SIMD2<Float>(Float(pts[i + stride].x), Float(pts[i + stride].y))
      let d = b - a
      let len = simd_length(d)
      if len > 1e-3 {
        let mid = (a + b) / 2
        inst.append(Instance(model: .translation([mid.x, mid.y, z]) * .rotZ(atan2(d.y, d.x)) * .scale([len + width * 0.5, width, 1]),
                             color: color))
      }
      i += stride
    }
    draw(enc, quad, inst)
  }

  private func draw(_ enc: MTLRenderCommandEncoder, _ mesh: Mesh, _ instances: [Instance]) {
    guard !instances.isEmpty else { return }
    let bytes = MemoryLayout<Instance>.stride * instances.count
    enc.setVertexBuffer(mesh.vertices, offset: 0, index: 0)
    if bytes <= 4096 {
      instances.withUnsafeBytes { enc.setVertexBytes($0.baseAddress!, length: bytes, index: 2) }
    } else {
      guard let buf = device.makeBuffer(bytes: instances, length: bytes, options: .storageModeShared) else { return }
      enc.setVertexBuffer(buf, offset: 0, index: 2)
    }
    enc.drawIndexedPrimitives(type: .triangle, indexCount: mesh.count, indexType: .uint16, indexBuffer: mesh.indices,
                              indexBufferOffset: 0, instanceCount: instances.count)
  }
}

// MARK: - SwiftUI host

final class SimMTKView: MTKView {
  var renderer: Renderer?
  override var acceptsFirstResponder: Bool { true }

  override func mouseDragged(with event: NSEvent) {
    guard let r = renderer else { return }
    MainActor.assumeIsolated {
      if r.options.camera == .chase { r.options.camera = .orbit }
      r.orbitAzimuth -= Float(event.deltaX) * 0.01
      r.orbitElevation = simd_clamp(r.orbitElevation + Float(event.deltaY) * 0.01, 0.05, 1.5)
    }
  }

  override func scrollWheel(with event: NSEvent) {
    guard let r = renderer else { return }
    MainActor.assumeIsolated {
      r.distance = simd_clamp(r.distance * (1 - Float(event.scrollingDeltaY) * 0.01), 3, 80)
    }
  }
}

import SwiftUI

struct MetalViewport: NSViewRepresentable {
  let engine: SimEngine
  var options: ViewOptions

  func makeNSView(context: Context) -> SimMTKView {
    let view = SimMTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
    view.colorPixelFormat = Renderer.colorFormat
    view.depthStencilPixelFormat = Renderer.depthFormat
    view.sampleCount = Renderer.sampleCount
    view.clearColor = Renderer.clearColor
    view.preferredFramesPerSecond = 60
    if let device = view.device, let r = Renderer(device: device) {
      r.engine = engine
      r.options = options
      view.renderer = r
      view.delegate = r
    }
    return view
  }

  func updateNSView(_ view: SimMTKView, context: Context) {
    guard let r = view.renderer else { return }
    // Mouse orbiting may have switched the camera; only push explicit changes.
    let cam = r.options.camera
    r.options = options
    if context.coordinator.lastCamera == options.camera { r.options.camera = cam }
    context.coordinator.lastCamera = options.camera
  }

  func makeCoordinator() -> Coordinator { Coordinator() }
  final class Coordinator { var lastCamera: CameraMode? }
}
