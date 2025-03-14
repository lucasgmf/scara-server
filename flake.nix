{
  description = "C Template";

  inputs = {
    nixpkgs.url = "nixpkgs";
    systems.url = "github:nix-systems/x86_64-linux";
    flake-utils = {
      url = "github:numtide/flake-utils";
      inputs.systems.follows = "systems";
    };
    pico-sdk = {
      url = "github:raspberrypi/pico-sdk";
      flake = false;
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        pname = "hello-world";
        version = "0.0.1";
        src = ./.;

        picoSdkPath = pkgs.fetchFromGitHub {
          owner = "raspberrypi";
          repo = "pico-sdk";
          rev = "ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c";
          sha256 = "sha256-wfvsDfzxkKImD7rFbqxD/+53WLebLuRpG91fjiLYDxY=";
          fetchSubmodules = true;
        };

        buildInputs = with pkgs; [
          picotool
        ];
        nativeBuildInputs = with pkgs; [
          cmake
          pkg-config
          clang-tools
          gcc-arm-embedded
          python3
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          inherit buildInputs nativeBuildInputs;
          shellHook = ''
            export PICO_SDK_PATH="${picoSdkPath}"
            export PICO_TINYUSB_PATH="${picoSdkPath}/lib/tinyusb"
            echo "PICO_SDK_PATH set to $PICO_SDK_PATH"
          '';
        };

        packages.default = pkgs.stdenv.mkDerivation {
          inherit
            buildInputs
            nativeBuildInputs
            pname
            version
            src
            ;
        };
      }
    );
}
