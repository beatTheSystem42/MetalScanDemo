//
//  ViewController.swift
//  MetalScanDemo
//
//  Created by Quentin Reiser on 9/2/21.
//

import Foundation
import UIKit
import ARKit
import MetalKit

class ViewController: UIViewController, MTKViewDelegate, ARSessionDelegate, ARSCNViewDelegate {
    
    var state = 0
    var session: ARSession!
    var wConfig: ARWorldTrackingConfiguration!
    var sConfig: ARWorldTrackingConfiguration!
    var renderer: Renderer!
    var mtkView: MTKView!
    
    var arView: ARSCNView!
    
    var sButt: UIButton!
    var tLabel: UILabel!
    var bLabel: UILabel!
    var colors = Colors()
    var tView: UIImageView!
    
    
    override func viewDidLoad() {
        
        super.viewDidLoad()

        session = ARSession()
        session.delegate = self
        
        wConfig = ARWorldTrackingConfiguration()
        wConfig.sceneReconstruction = .mesh
        
        mtkView = MTKView(frame: view.frame)
        view.addSubview(mtkView)

        mtkView.device = MTLCreateSystemDefaultDevice()
        mtkView.backgroundColor = .black
        mtkView.delegate = self

        renderer = Renderer(session: session, view: mtkView)
        renderer.vc = self
        
        setupOverlay()
    }
    
        
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)

        session.run(wConfig)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        session.pause()
    }
    
    
    // button control flow
    @objc func sBDown() {
        
    }
    
    @objc func sBTouched() {
        
        switch state {
        case 0:
            state = 1
            sButt.setTitle("done", for: .normal)
        case 1:
            state = 3
            session.pause()
            setupScanView()
        default:
            break
        }
    }
    
    // you can do a single node for each mesh node, but in my app I make a node
    // for each face since i need to apply a different camera texture to each
    func makeTexturedMesh() {
        
        let scanMeshes = renderer.scanMeshes
        let textureCloud = renderer.textureCloud
        
        for mesh in scanMeshes {
            
            // in my app i determine what camera frame to use based on the normals
            // per vertex, but for simplicity here, just use the first texture saved
            
            let frame = textureCloud["0"]!.frame
            let aTrans = SCNMatrix4(mesh.transform)
            
            let vertices: ARGeometrySource = mesh.vertices
            let normals: ARGeometrySource = mesh.normals
            let faces: ARGeometryElement = mesh.submesh
            
            let texture = getTextureImage(frame: frame)
            
            for f in 0..<faces.count {
                // check to see if each vertex is visible aka scanned
                var c = 0
                let face = face(at: f, faces: faces)
                for fv in face {
                    if mesh.inBox[fv] == 1 {
                        c += 1
                    }
                }
                
                if c == 3 {
                    // all verts are visible, so the face is visible
                    var fVerts: [SCNVector3] = []
                    var fNorms: [SCNVector3] = []
                    var tCoords: [vector_float2] = []
                    
                    for fv in face {
                        let vert = vertex(at: UInt32(fv), vertices: vertices)
                        let vTrans = SCNMatrix4MakeTranslation(vert[0], vert[1], vert[2])
                        let wTrans = SCNMatrix4Mult(vTrans, aTrans)
                        let wPos = SCNVector3(wTrans.m41, wTrans.m42, wTrans.m43)
                        fVerts.append(wPos)
                        
                        let norm = normal(at: UInt32(fv), normals: normals)
                        let nTrans = SCNMatrix4MakeTranslation(norm[0], norm[1], norm[2])
                        let wNTrans = SCNMatrix4Mult(nTrans, aTrans)
                        let wNPos = SCNVector3(wNTrans.m41, wTrans.m42, wNTrans.m43)
                        fNorms.append(wNPos)
                        
                        let tCoord = getTextureCoord(frame: frame, vert: vert, aTrans: mesh.transform)
                        tCoords.append(tCoord)
                        
                        // visualize the normals with cylinders if you want
                        
                        if mesh.inBox[fv] == 1 {
                            //let normVis = lineBetweenNodes(positionA: wPos, positionB: wNPos, inScene: arView.scene)
                            //arView.scene.rootNode.addChildNode(normVis)
                        }
                    }
                    
                    let vertsSource = SCNGeometrySource(vertices: fVerts)
                    let normsSource = SCNGeometrySource(normals: fNorms)
                    let facesSource = SCNGeometryElement(indices: [UInt32(0), UInt32(1), UInt32(2)], primitiveType: .triangles)
                    
                    let textrSource = SCNGeometrySource(textureCoordinates: tCoords)
                    
                    let geom = SCNGeometry(sources: [vertsSource, normsSource, textrSource], elements: [facesSource])
                    let mat = SCNMaterial()
                    mat.diffuse.contents = texture
                    mat.isDoubleSided = false
                    geom.materials = [mat]
                    let meshNode = SCNNode(geometry: geom)
                    
                    DispatchQueue.main.async {
                        self.arView.scene.rootNode.addChildNode(meshNode)
                    }
                    
                }
            }
        }
    }
    
    
    
    func getTextureCoord(frame: ARFrame, vert: SIMD3<Float>, aTrans: simd_float4x4) -> vector_float2 {
        
        let cam = frame.camera
        let size = cam.imageResolution
        let vertex4 = vector_float4(vert.x, vert.y, vert.z, 1)
        let world_vertex4 = simd_mul(aTrans, vertex4)
        let world_vector3 = simd_float3(x: world_vertex4.x, y: world_vertex4.y, z: world_vertex4.z)
        let pt = cam.projectPoint(world_vector3,
            orientation: .portrait,
            viewportSize: CGSize(
                width: CGFloat(size.height),
                height: CGFloat(size.width)))
        let v = 1.0 - Float(pt.x) / Float(size.height)
        let u = Float(pt.y) / Float(size.width)
        
        let tCoord = vector_float2(u, v)
        
        return tCoord
    }
    
    func getTextureCoords(frame: ARFrame, vertices: ARGeometrySource, aTrans: simd_float4x4) -> [vector_float2] {
        
        var tCoords: [vector_float2] = []
        
        for v in 0..<vertices.count {
            let vert = vertex(at: UInt32(v), vertices: vertices)
            let tCoord = getTextureCoord(frame: frame, vert: vert, aTrans: aTrans)
            
            tCoords.append(tCoord)
        }
        
        return tCoords
    }
    
    func getTextureImage(frame: ARFrame) -> UIImage? {

        let pixelBuffer = frame.capturedImage
        let image = CIImage(cvPixelBuffer: pixelBuffer)
        
        let context = CIContext(options:nil)
        guard let cameraImage = context.createCGImage(image, from: image.extent) else {return nil}

        return UIImage(cgImage: cameraImage)
    }
    
    
    func vertex(at index: UInt32, vertices: ARGeometrySource) -> SIMD3<Float> {
        assert(vertices.format == MTLVertexFormat.float3, "Expected three floats (twelve bytes) per vertex.")
        let vertexPointer = vertices.buffer.contents().advanced(by: vertices.offset + (vertices.stride * Int(index)))
        let vertex = vertexPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
        return vertex
    }
    
    func normal(at index: UInt32, normals: ARGeometrySource) -> SIMD3<Float> {
        assert(normals.format == MTLVertexFormat.float3, "Expected three floats (twelve bytes) per normal.")
        let normalPointer = normals.buffer.contents().advanced(by: normals.offset + (normals.stride * Int(index)))
        let normal = normalPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
        return normal
    }
    
    func face(at index: Int, faces: ARGeometryElement) -> [Int] {
        let indicesPerFace = faces.indexCountPerPrimitive
        let facesPointer = faces.buffer.contents()
        var vertexIndices = [Int]()
        for offset in 0..<indicesPerFace {
            let vertexIndexAddress = facesPointer.advanced(by: (index * indicesPerFace + offset) * MemoryLayout<UInt32>.size)
            vertexIndices.append(Int(vertexIndexAddress.assumingMemoryBound(to: UInt32.self).pointee))
        }
        return vertexIndices
    }
    
    
    // draws a cylinder for a 3D line
    func lineBetweenNodes(positionA: SCNVector3, positionB: SCNVector3, inScene: SCNScene) -> SCNNode {
        let vector = SCNVector3(positionA.x - positionB.x, positionA.y - positionB.y, positionA.z - positionB.z)
        let dist = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
        let mid = SCNVector3(x:(positionA.x + positionB.x) / 2, y:(positionA.y + positionB.y) / 2, z:(positionA.z + positionB.z) / 2)
        
        // if you want it shorter use quarter & make height / 2
        let quarter = SCNVector3(x:(positionA.x + mid.x) / 2, y:(positionA.y + mid.y) / 2, z:(positionA.z + mid.z) / 2)

        let lineGeometry = SCNCylinder()
        lineGeometry.radius = 0.001
        lineGeometry.height = CGFloat(dist / 2)
        lineGeometry.radialSegmentCount = 5
        lineGeometry.firstMaterial!.diffuse.contents = UIColor.magenta

        let lineNode = SCNNode(geometry: lineGeometry)
        lineNode.position = quarter
        lineNode.look (at: mid, up: inScene.rootNode.worldUp, localFront: lineNode.worldUp)
        lineNode.opacity = 0.6
        return lineNode
    }
    
    
    func setupScanView() {
        arView = ARSCNView(frame: view.frame)
        arView.delegate = self
        arView.autoenablesDefaultLighting = false
        
        sConfig = ARWorldTrackingConfiguration()
        sConfig.planeDetection = [.horizontal, .vertical]
        arView.session.run(sConfig, options: [])
        view.addSubview(arView)
        
        let tVWidth = view.frame.width * 0.12
        let tVHeight = view.frame.height * 0.12
        let tVRect = CGRect(x: 0, y: 0, width: tVWidth, height: tVHeight)
        tView = UIImageView(frame: tVRect)
        tView.backgroundColor = UIColor.white
        arView.addSubview(tView)
        
        makeTexturedMesh()
    }
    
    
    func setupOverlay() {
        
        let tlWidth = view.frame.width * 0.88
        let tlHeight = tlWidth * 0.18
        let tlX = view.frame.width * 0.5 - (tlWidth / 2)
        let tlY = view.frame.height * 0.14
        let tlRect = CGRect(x: tlX, y: tlY, width: tlWidth, height: tlHeight)
        tLabel = UILabel(frame: tlRect)
        tLabel.font = UIFont(name: "Avenir", size: 18)
        tLabel.textAlignment = .center
        tLabel.adjustsFontSizeToFitWidth = true
        tLabel.textColor = colors.realLightColor
        tLabel.text = "hello"
        view.addSubview(tLabel)
        
        let blY = tLabel.frame.maxY
        let blRect = CGRect(x: tlX, y: blY, width: tlWidth, height: tlHeight)
        bLabel = UILabel(frame: blRect)
        bLabel.font = UIFont(name: "Avenir", size: 18)
        bLabel.textAlignment = .center
        bLabel.adjustsFontSizeToFitWidth = true
        bLabel.textColor = colors.realLightColor
        bLabel.text = "ready to scan"
        view.addSubview(bLabel)
        
        let bWidth = view.frame.width * 0.32
        let bHeight = bWidth * 0.44
        let bX = view.frame.width * 0.5 - (bWidth / 2)
        let bY = view.frame.height * 0.9
        let bRect = CGRect(x: bX, y: bY, width: bWidth, height: bHeight)
        sButt = UIButton(frame: bRect)
        sButt.alpha = 1.0
        sButt.isEnabled = true
        sButt.layer.cornerRadius = bHeight / 2
        sButt.backgroundColor = colors.blueColor
        sButt.titleLabel?.font = UIFont(name: "Avenir", size: 20)
        sButt.setTitleColor(colors.lightColor, for: .normal)
        sButt.setTitle("scan", for: .normal)
        sButt.addTarget(self, action: #selector(sBDown), for: .touchDown)
        sButt.addTarget(self, action: #selector(sBTouched), for: .touchUpInside)
        view.addSubview(sButt)
    }

    // MARK: - MTKViewDelegate

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
    }

    func draw(in view: MTKView) {
        renderer.update()
    }
    
    override var prefersHomeIndicatorAutoHidden: Bool {return true}
    override var prefersStatusBarHidden: Bool {return true}
}


// MARK: Extensions & HELPERS

extension ARMeshGeometry {
    func vertex(at index: UInt32) -> SIMD3<Float> {
        assert(vertices.format == MTLVertexFormat.float3, "Expected three floats (twelve bytes) per vertex.")
        let vertexPointer = vertices.buffer.contents().advanced(by: vertices.offset + (vertices.stride * Int(index)))
        let vertex = vertexPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
        return vertex
    }
    
    func normal(at index: UInt32) -> SIMD3<Float> {
        assert(normals.format == MTLVertexFormat.float3, "Expected three floats (twelve bytes) per normal.")
        let normalPointer = normals.buffer.contents().advanced(by: normals.offset + (normals.stride * Int(index)))
        let normal = normalPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
        return normal
    }
    
    func vertexIndicesOf(faceWithIndex index: Int) -> [Int] {
        let indicesPerFace = faces.indexCountPerPrimitive
        let facesPointer = faces.buffer.contents()
        var vertexIndices = [Int]()
        for offset in 0..<indicesPerFace {
            let vertexIndexAddress = facesPointer.advanced(by: (index * indicesPerFace + offset) * MemoryLayout<UInt32>.size)
            vertexIndices.append(Int(vertexIndexAddress.assumingMemoryBound(to: UInt32.self).pointee))
        }
        return vertexIndices
    }
}

extension  SCNGeometrySource {
    convenience init(_ source: ARGeometrySource, semantic: Semantic) {
           self.init(buffer: source.buffer, vertexFormat: source.format, semantic: semantic, vertexCount: source.count, dataOffset: source.offset, dataStride: source.stride)
    }
    
    convenience init(textureCoordinates texcoord: [vector_float2]) {
        let stride = MemoryLayout<vector_float2>.stride
        let bytePerComponent = MemoryLayout<Float>.stride
        let data = Data(bytes: texcoord, count: stride * texcoord.count)
        self.init(data: data, semantic: SCNGeometrySource.Semantic.texcoord, vectorCount: texcoord.count, usesFloatComponents: true, componentsPerVector: 2, bytesPerComponent: bytePerComponent, dataOffset: 0, dataStride: stride)
    }
}
extension  SCNGeometryElement {
    convenience init(_ source: ARGeometryElement) {
        let pointer = source.buffer.contents()
        let byteCount = source.count * source.indexCountPerPrimitive * source.bytesPerIndex
        let data = Data(bytesNoCopy: pointer, count: byteCount, deallocator: .none)
        self.init(data: data, primitiveType: .of(source.primitiveType), primitiveCount: source.count, bytesPerIndex: source.bytesPerIndex)
    }
}

extension  SCNGeometryPrimitiveType {
    static  func  of(_ type: ARGeometryPrimitiveType) -> SCNGeometryPrimitiveType {
       switch type {
       case .line:
               return .line
       case .triangle:
               return .triangles
       }
    }
}


struct BoundingBox {
  let min: SCNVector3
  let max: SCNVector3

  init(_ boundTuple: (min: SCNVector3, max: SCNVector3)) {
    min = boundTuple.min
    max = boundTuple.max
  }

  func contains(_ point: SCNVector3) -> Bool {
    let contains =
    min.x <= point.x &&
    min.y <= point.y &&
    min.z <= point.z &&

    max.x > point.x &&
    max.y > point.y &&
    max.z > point.z

    return contains
  }
}


