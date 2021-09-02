//
//  Renderer.swift
//  MetalScanDemo
//
//  Created by Quentin Reiser on 9/2/21.
//

import Foundation
import Metal
import MetalKit
import ARKit

struct FrameUniforms {
    var projectionMatrix: float4x4
    var viewMatrix: float4x4
}

struct FragmentUniforms {
    var ambientLightColor: SIMD3<Float>
    var directionalLightDirection: SIMD3<Float>
    var directionalLightColor: SIMD3<Float>
    var materialShininess: Float
}

struct InstanceUniforms {
    var modelMatrix: float4x4
}

struct ScanMesh {
    let transform: float4x4
    let vertices: ARGeometrySource
    let normals: ARGeometrySource
    let submesh: ARGeometryElement
    let inBox: [Int32]
}

let kMaxBuffersInFlight: Int = 3

let kMaxAnchorInstanceCount: Int = 64

let kAlignedFrameUniformsSize = 256
let kAlignedFragmentUniformsSize = 256
let kAlignedInstanceUniformsSize = 16_384

let kImagePlaneVertexData: [Float] = [
    -1.0, -1.0,  0.0, 1.0,
     1.0, -1.0,  1.0, 1.0,
    -1.0,  1.0,  0.0, 0.0,
     1.0,  1.0,  1.0, 0.0,
]

class Renderer {
    
    var vc: ViewController!
    
    let session: ARSession
    let device: MTLDevice
    let inFlightSemaphore = DispatchSemaphore(value: kMaxBuffersInFlight)
    var mtkView: MTKView

    var commandQueue: MTLCommandQueue!
    var frameUniformBuffer: MTLBuffer!
    var anchorUniformBuffer: MTLBuffer!
    var fragmentUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!
    var cubeVertexBuffer: MTLBuffer!
    var cubeNormalBuffer: MTLBuffer!
    var cubeTransformBuffer: MTLBuffer!
    
    var cameraPipelineState: MTLRenderPipelineState!
    var anchorPipelineState: MTLRenderPipelineState!
    var outlinePipelineState: MTLRenderPipelineState!
    
    var cameraDepthState: MTLDepthStencilState!
    var anchorDepthState: MTLDepthStencilState!

    var cameraTextureY: CVMetalTexture?
    var cameraTextureCbCr: CVMetalTexture?
    var textureCache: CVMetalTextureCache!
    
    var vertexDescriptor: MTLVertexDescriptor!

    var scanMeshes: [ScanMesh] = []
    var bBox: BoundingBox!
    var bBoxOrigin: SCNVector3!

    var uniformBufferIndex: Int = 0
    var frameUniformBufferOffset: Int = 0
    var anchorUniformBufferOffset: Int = 0
    var frameUniformBufferAddress: UnsafeMutableRawPointer!
    var anchorUniformBufferAddress: UnsafeMutableRawPointer!
    var anchorInstanceCount: Int = 0
    var viewportSize: CGSize = CGSize()
    
    var textureCloud: [String: TextureFrame] = [:]
    
    struct TextureFrame {
        var key: String       // pie slice
        var distance: CGFloat // dist from bbox
        var frame: ARFrame    // saved frame
    }

    init(session: ARSession, view: MTKView) {
        self.session = session
        self.device = view.device!
        self.mtkView = view
        loadMetal()
    }

    func update() {
        let _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)

        if let commandBuffer = commandQueue.makeCommandBuffer() {
            var textures = [cameraTextureY, cameraTextureCbCr]
            commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
                if let strongSelf = self {
                    strongSelf.inFlightSemaphore.signal()
                }
                textures.removeAll()
            }
            
            updateBufferStates()
            updateFrameState()
            
            if let renderPassDescriptor = mtkView.currentRenderPassDescriptor, let currentDrawable = mtkView.currentDrawable, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {

                drawCameraImage(renderEncoder: renderEncoder)
                if vc.state == 1 {
                    drawAnchorGeometry(renderEncoder: renderEncoder)
                }

                renderEncoder.endEncoding()

                commandBuffer.present(currentDrawable)
            }

            commandBuffer.commit()
        }
    }
    
    
    
    // MARK: UPDATING MESH
    
    func updateScanMeshAnchors(_ frame: ARFrame) {
        let anchors = frame.anchors.filter { $0 is ARMeshAnchor } as! [ARMeshAnchor]
        
        if textureCloud.count < 6 {
            checkShouldSave(frame: frame)
        }
        
        scanMeshes = anchors.map { anchor in
            let aTrans = SCNMatrix4(anchor.transform)
            
            let meshGeometry = anchor.geometry
            let vertices: ARGeometrySource = meshGeometry.vertices
            let normals: ARGeometrySource = meshGeometry.normals
            let submesh: ARGeometryElement = meshGeometry.faces
            var inBox: [Int32] = []
            
            for vIndex in 0..<vertices.count {
                let vertex = meshGeometry.vertex(at: UInt32(vIndex))
                let vTrans = SCNMatrix4MakeTranslation(vertex[0], vertex[1], vertex[2])
                let wTrans = SCNMatrix4Mult(vTrans, aTrans)
                let wPos = SCNVector3(wTrans.m41, wTrans.m42, wTrans.m43)
                
                // check to see if the vertex is withing the bBox
                if bBox.contains(wPos) {
                    inBox.append(1)
                } else {
                    inBox.append(0)
                }
            }
            let scanMesh = ScanMesh(transform: anchor.transform,
                                      vertices: vertices,
                                      normals: normals,
                                      submesh: submesh,
                                      inBox: inBox)
            return scanMesh
        }
    }
    
    
    // this function determines if we should save a frame to use for texturing later
    // this is a simplified 2D approach
    // think about looking down from way up on the Y-axis
    // imagine a circle with the origin being the center of the bBox you are scanning
    // this determines what 'pie slice' the camera is in and saves one per slice
    // there are many different ways to do this, this is one
    func checkShouldSave(frame: ARFrame) {
        
        // camera position
        let camTrans = frame.camera.transform
        let camPos = SCNVector3(camTrans.columns.3.x, camTrans.columns.3.y, camTrans.columns.3.z)
        
        // find polar coordinates around bBox
        let camPos2D = CGPoint(x: CGFloat(camPos.x), y: CGFloat(camPos.z))
        let bOrigin2D = CGPoint(x: CGFloat(bBoxOrigin.x), y: CGFloat(bBoxOrigin.z))
        let cam2Box = CGPoint(x: camPos2D.x - bOrigin2D.x, y: camPos2D.y - bOrigin2D.y)
        let quad = cameraQuad(origin: bOrigin2D, camPos: cam2Box)
        let camAngle2D = atan(cam2Box.y / cam2Box.x)
        let slice = pieSlice(angle: camAngle2D, quad: quad)
        vc.tLabel.text = "camera in pie slice: \(slice)"
        
        
        // rudimentary check to see if bBox is in camera viewport
        var boxIsVisible = false
        
        let bBox2Screen = frame.camera.projectPoint(simd_float3(bBoxOrigin), orientation: .portrait, viewportSize: mtkView.frame.size)
        
        if bBox2Screen.x > 0 && bBox2Screen.y > 0 {
            if bBox2Screen.x <= mtkView.frame.size.width
                && bBox2Screen.y <= mtkView.frame.size.height {
                boxIsVisible = true
            } else {
                boxIsVisible = false
            }
        } else {
            boxIsVisible = false
        }
        
        // check if the bBox isn't too far or too close
        let dist = dist3D(a: camPos, b: bBoxOrigin)
        
        if dist > 0.78 && dist < 2.5 && boxIsVisible {
            if textureCloud[slice.description] == nil {
                vc.bLabel.text = "saved frame for slice: \(slice)"
                let textFrame = TextureFrame(key: slice.description, distance: dist, frame: frame)
                textureCloud[slice.description] = textFrame
            } else {
                // check if it's a better frame,
                // or if more than one should be saved?
            }
        }
    }
    
    
    // which quad (in bBox local space) is the camera in?
    func cameraQuad(origin: CGPoint, camPos: CGPoint) -> Int {
        if camPos.x <= 0 {
            if camPos.y <= 0 {
                return 1
            } else {
                return 2
            }
        } else {
            if camPos.y <= 0 {
                return 0
            } else {
                return 3
            }
        }
    }
    
    // which pie slice (in bBox local space) is the camera in?
    func pieSlice(angle: CGFloat, quad: Int) -> Int {
    
        let angle = abs(angle)
        var a = 0
        var i: Int = 12
        
        if angle <= 0.5236 {
            a = 0
        }
        if angle > 0.5236 && angle <= 1.0471 {
            a = 1
        }
        if angle > 1.0471 {
            a = 2
        }
        
        switch quad {
        case 0:
            i = a - 0
        case 1:
            i = 5 - a
        case 2:
            i = 6 + a
        case 3:
            i = 11 - a
        default:
            break
        }
        
        // originally had 12 slices, made it 6 for simplicity
        // however, i haven't fixed this quick patch job yet
        switch i {
        case 0:
            i = 4
        case 1:
            i = 4
        case 2:
            i = 3
        case 3:
            i = 3
        case 4:
            i = 2
        case 5:
            i = 2
        case 6:
            i = 1
        case 7:
            i = 1
        case 8:
            i = 0
        case 9:
            i = 0
        case 10:
            i = 5
        case 11:
            i = 5
        default:
            break
            
        }
        
        return i
    }
    
    
    func dist3D(a: SCNVector3, b: SCNVector3) -> CGFloat {
        let dist = sqrt(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y)) + ((b.z - a.z) * (b.z - a.z)))
        return CGFloat(dist)
    }
    
    func mid3D(a: SCNVector3, b: SCNVector3) -> SCNVector3 {
        let x = (a.x + b.x) / 2
        let y = (a.y + b.y) / 2
        let z = (a.z + b.z) / 2
        
        return SCNVector3Make(x, y, z)
    }
    
    
    // MARK: - METAL PIPELINE
    
    func loadMetal() {
        
        mtkView.depthStencilPixelFormat = .depth32Float_stencil8
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.sampleCount = 1

        let frameUniformBufferSize = kAlignedFrameUniformsSize * kMaxBuffersInFlight
        let anchorUniformBufferSize = kAlignedInstanceUniformsSize * kMaxBuffersInFlight

        frameUniformBuffer = device.makeBuffer(length: frameUniformBufferSize, options: .storageModeShared)
        
        fragmentUniformBuffer = device.makeBuffer(length: kAlignedFragmentUniformsSize, options: .storageModeShared)
        
        anchorUniformBuffer = device.makeBuffer(length: anchorUniformBufferSize, options: .storageModeShared)
        
        let imagePlaneVertexDataCount = kImagePlaneVertexData.count * MemoryLayout<Float>.size
        imagePlaneVertexBuffer = device.makeBuffer(bytes: kImagePlaneVertexData, length: imagePlaneVertexDataCount, options: [])

        let defaultLibrary = device.makeDefaultLibrary()!
        
        let cameraVertexFunction = defaultLibrary.makeFunction(name: "cameraVertexTransform")!
        let cameraFragmentFunction = defaultLibrary.makeFunction(name: "cameraFragmentShader")!

        let imagePlaneVertexDescriptor = MTLVertexDescriptor()
        imagePlaneVertexDescriptor.attributes[0].format = .float2
        imagePlaneVertexDescriptor.attributes[0].offset = 0
        imagePlaneVertexDescriptor.attributes[0].bufferIndex = 0
        imagePlaneVertexDescriptor.attributes[1].format = .float2
        imagePlaneVertexDescriptor.attributes[1].offset = 8
        imagePlaneVertexDescriptor.attributes[1].bufferIndex = 0
        imagePlaneVertexDescriptor.layouts[0].stride = 16

        let cameraPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        cameraPipelineStateDescriptor.sampleCount = mtkView.sampleCount
        cameraPipelineStateDescriptor.vertexFunction = cameraVertexFunction
        cameraPipelineStateDescriptor.fragmentFunction = cameraFragmentFunction
        cameraPipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        cameraPipelineStateDescriptor.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat
        cameraPipelineStateDescriptor.depthAttachmentPixelFormat = mtkView.depthStencilPixelFormat
        cameraPipelineStateDescriptor.stencilAttachmentPixelFormat = mtkView.depthStencilPixelFormat
        
        do {
            try cameraPipelineState = device.makeRenderPipelineState(descriptor: cameraPipelineStateDescriptor)
        } catch let error {
            print("Failed to created captured image pipeline state, error \(error)")
        }
        
        let cameraDepthStateDescriptor = MTLDepthStencilDescriptor()
        cameraDepthStateDescriptor.depthCompareFunction = .always
        cameraDepthStateDescriptor.isDepthWriteEnabled = false
        cameraDepthState = device.makeDepthStencilState(descriptor: cameraDepthStateDescriptor)

        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)
        self.textureCache = textureCache
        
        let anchorGeometryVertexFunction = defaultLibrary.makeFunction(name: "anchorGeometryVertexTransform")!
        let anchorGeometryFragmentFunction = defaultLibrary.makeFunction(name: "anchorGeometryFragmentLighting")!
        
        vertexDescriptor = MTLVertexDescriptor()
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        vertexDescriptor.attributes[1].format = .float3
        vertexDescriptor.attributes[1].offset = 0
        vertexDescriptor.attributes[1].bufferIndex = 1
        vertexDescriptor.layouts[0].stride = 12
        vertexDescriptor.layouts[1].stride = 12

        let anchorPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        anchorPipelineStateDescriptor.sampleCount = mtkView.sampleCount
        anchorPipelineStateDescriptor.vertexFunction = anchorGeometryVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = anchorGeometryFragmentFunction
        anchorPipelineStateDescriptor.vertexDescriptor = vertexDescriptor
        anchorPipelineStateDescriptor.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat
        anchorPipelineStateDescriptor.colorAttachments[0].isBlendingEnabled = true
        anchorPipelineStateDescriptor.colorAttachments[0].rgbBlendOperation = .add
        anchorPipelineStateDescriptor.colorAttachments[0].alphaBlendOperation = .add
        anchorPipelineStateDescriptor.colorAttachments[0].sourceRGBBlendFactor = .one
        anchorPipelineStateDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        anchorPipelineStateDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        anchorPipelineStateDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        anchorPipelineStateDescriptor.depthAttachmentPixelFormat = mtkView.depthStencilPixelFormat
        anchorPipelineStateDescriptor.stencilAttachmentPixelFormat = mtkView.depthStencilPixelFormat
        
        do {
            try anchorPipelineState = device.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor)
        } catch let error {
            print("Failed to create anchor geometry pipeline state, error \(error)")
        }
        
        
        // place an example bounding box of size 1 at (0,0,-1)
        // this won't be rendered, but anything scanned inside of it will
        let min = SCNVector3(-0.5, -0.5, -1.5)
        let max = SCNVector3(0.5, 0.5, -0.5)
        bBox = BoundingBox((min: min, max: max))
        bBoxOrigin = SCNVector3(0, 0, -1)
        
        
        let geometryOutlineVertexFunction = defaultLibrary.makeFunction(name: "anchorGeometryVertexTransform")!
        let geometryOutlineFragmentFunction = defaultLibrary.makeFunction(name: "geometryOutlineFragment")!
        anchorPipelineStateDescriptor.vertexFunction = geometryOutlineVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = geometryOutlineFragmentFunction

        do {
            try outlinePipelineState = device.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor)
        } catch let error {
            print("Failed to create outline geometry pipeline state, error \(error)")
        }

        
        let anchorDepthStateDescriptor = MTLDepthStencilDescriptor()
        anchorDepthStateDescriptor.depthCompareFunction = .lessEqual
        anchorDepthStateDescriptor.isDepthWriteEnabled = true
        anchorDepthState = device.makeDepthStencilState(descriptor: anchorDepthStateDescriptor)

        commandQueue = device.makeCommandQueue()
    }

    
    func updateBufferStates() {
        uniformBufferIndex = (uniformBufferIndex + 1) % kMaxBuffersInFlight
        
        frameUniformBufferOffset = kAlignedFrameUniformsSize * uniformBufferIndex
        anchorUniformBufferOffset = kAlignedInstanceUniformsSize * uniformBufferIndex
        
        frameUniformBufferAddress = frameUniformBuffer.contents().advanced(by: frameUniformBufferOffset)
        anchorUniformBufferAddress = anchorUniformBuffer.contents().advanced(by: anchorUniformBufferOffset)
    }
    
    func updateFrameState() {
        guard let currentFrame = session.currentFrame else {
            return
        }
        
        viewportSize = mtkView.drawableSize
        updateImagePlane(frame: currentFrame)

        if vc.state == 1 {
            updateScanMeshAnchors(currentFrame)
        }
        updateFrameUniforms(frame: currentFrame)
        updateAnchors(frame: currentFrame)
        updateCameraTextures(frame: currentFrame)
    }
    
    func updateFrameUniforms(frame: ARFrame) {
        let uniforms = frameUniformBufferAddress.assumingMemoryBound(to: FrameUniforms.self)
        
        uniforms.pointee.viewMatrix = frame.camera.viewMatrix(for: .portrait)
        uniforms.pointee.projectionMatrix = frame.camera.projectionMatrix(for: .portrait,
                                                                          viewportSize: viewportSize,
                                                                          zNear: 0.05,
                                                                          zFar: 50)
    }
    
    // passing in transforms
    func updateAnchors(frame: ARFrame) {
        for (index, mesh) in scanMeshes.enumerated() {
            let instanceIndex = min(index, kMaxAnchorInstanceCount - 1)
            let modelMatrix = mesh.transform
            let anchorUniforms = anchorUniformBufferAddress.assumingMemoryBound(to: InstanceUniforms.self).advanced(by: instanceIndex)
            anchorUniforms.pointee.modelMatrix = modelMatrix
        }
    }
    
    
    // background camera image
    func updateCameraTextures(frame: ARFrame) {
        let pixelBuffer = frame.capturedImage
        cameraTextureY = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.r8Unorm, planeIndex:0)
        cameraTextureCbCr = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.rg8Unorm, planeIndex:1)
    }
    
    func createTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        
        var texture: CVMetalTexture? = nil
        _ = CVMetalTextureCacheCreateTextureFromImage(nil,
                                                      textureCache,
                                                      pixelBuffer,
                                                      nil,
                                                      pixelFormat,
                                                      width,
                                                      height,
                                                      planeIndex,
                                                      &texture)
        return texture
    }
    
    func updateImagePlane(frame: ARFrame) {
        let displayToCameraTransform = frame.displayTransform(for: .portrait, viewportSize: viewportSize).inverted()

        let vertexData = imagePlaneVertexBuffer.contents().assumingMemoryBound(to: Float.self)
        for index in 0...3 {
            let textureCoordIndex = 4 * index + 2
            let textureCoord = CGPoint(x: CGFloat(kImagePlaneVertexData[textureCoordIndex]),
                                       y: CGFloat(kImagePlaneVertexData[textureCoordIndex + 1]))
            let transformedCoord = textureCoord.applying(displayToCameraTransform)
            vertexData[textureCoordIndex] = Float(transformedCoord.x)
            vertexData[textureCoordIndex + 1] = Float(transformedCoord.y)
        }
    }
    
    
    // MARK: METAL DRAW
    
    func drawCameraImage(renderEncoder: MTLRenderCommandEncoder) {
        guard let textureY = cameraTextureY, let textureCbCr = cameraTextureCbCr else {
            return
        }

        renderEncoder.setCullMode(.none)
        renderEncoder.setRenderPipelineState(cameraPipelineState)
        renderEncoder.setDepthStencilState(cameraDepthState)
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
        renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(textureY), index: 1)
        renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(textureCbCr), index: 2)
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
    }
    
    func drawAnchorGeometry(renderEncoder: MTLRenderCommandEncoder) {

        for (index, mesh) in scanMeshes.enumerated() {
            renderEncoder.setVertexBuffer(mesh.vertices.buffer, offset: 0, index: 0)
            renderEncoder.setVertexBuffer(mesh.normals.buffer, offset: 0, index: 1)
            renderEncoder.setVertexBuffer(anchorUniformBuffer,
                                          offset: anchorUniformBufferOffset + MemoryLayout<InstanceUniforms>.size * index,
                                          index: 2)
            renderEncoder.setVertexBuffer(frameUniformBuffer, offset: frameUniformBufferOffset, index: 3)
            
            renderEncoder.setVertexBytes(mesh.inBox, length: MemoryLayout<Int32>.size * mesh.inBox.count, index: 4)
            renderEncoder.setRenderPipelineState(anchorPipelineState)
            renderEncoder.setDepthStencilState(anchorDepthState)
            renderEncoder.setTriangleFillMode(.fill)
            renderEncoder.drawIndexedPrimitives(type: .triangle,
                                                indexCount: mesh.submesh.count * mesh.submesh.indexCountPerPrimitive,
                                                indexType: .uint32,
                                                indexBuffer: mesh.submesh.buffer,
                                                indexBufferOffset: 0)

            renderEncoder.setRenderPipelineState(outlinePipelineState)
            renderEncoder.setDepthStencilState(anchorDepthState)
            renderEncoder.setTriangleFillMode(.lines)
            renderEncoder.drawIndexedPrimitives(type: .triangle,
                                                indexCount: mesh.submesh.count * mesh.submesh.indexCountPerPrimitive,
                                                indexType: .uint32,
                                                indexBuffer: mesh.submesh.buffer,
                                                indexBufferOffset: 0)
        }
    }
}

