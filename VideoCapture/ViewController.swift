//
//  ViewController.swift
//  VideoCapture
//
//  Created by Ryan Krienitz on 5/10/18.
//  Copyright © 2018 com. All rights reserved.
//

import UIKit

class ViewController: UIViewController, CameraBufferDelegate {
    
    var cameraBuffer: CameraBuffer!
    let opencvWrapper = OpenCVWrapper();
    @IBOutlet weak var imageView: UIImageView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        cameraBuffer = CameraBuffer()
        cameraBuffer.delegate = self
    }
    
    func captured(image: UIImage) {
        imageView.image = opencvWrapper.findHand(image)
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }


}

