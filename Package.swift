// swift-tools-version:6.0
// Builds the C++ core, its C API, the headless CLI, the test runner and the SwiftUI app.
// (CMakeLists.txt builds the same core/CLI/tests without Swift, e.g. for engine integration.)
import PackageDescription

let package = Package(
  name: "VehicleSim",
  platforms: [.macOS(.v14)],
  products: [
    .executable(name: "VehicleSimApp", targets: ["VehicleSimApp"]),
    .executable(name: "vdsim", targets: ["vdsim"]),
    .library(name: "VehicleCAPI", targets: ["VehicleCAPI"]),
  ],
  targets: [
    .target(
      name: "VehicleCore",
      path: "core",
      sources: ["src"],
      publicHeadersPath: "include"
    ),
    .target(
      name: "VehicleCAPI",
      dependencies: ["VehicleCore"],
      path: "capi",
      sources: ["vd_capi.cpp"],
      publicHeadersPath: "include"
    ),
    .executableTarget(
      name: "vdsim",
      dependencies: ["VehicleCore"],
      path: "cli"
    ),
    .executableTarget(
      name: "vdtests",
      dependencies: ["VehicleCore", "VehicleCAPI"],
      path: "tests"
    ),
    .executableTarget(
      name: "VehicleSimApp",
      dependencies: ["VehicleCAPI"],
      path: "app/VehicleSimApp"
    ),
  ],
  swiftLanguageModes: [.v5],
  cxxLanguageStandard: .cxx20
)
