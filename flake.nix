{
  description = "An exact and heuristic solver for directed feedback vertex set.";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/21.11";

    utils.url = "github:numtide/flake-utils";
    utils.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, ... }@inputs: inputs.utils.lib.eachSystem [
    "x86_64-linux" "i686-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin"
  ] (system:
  let pkgs = import nixpkgs { inherit system; };
      base-shell = (additionalPackages: pkgs.mkShell rec {
        name = "doom";

        hardeningDisable = [ "all" ];

        packages = additionalPackages ++ (with pkgs; [
          # Development Tools
          gcc.out
          cmake
          cmakeCurses

          # development dependencies
          graphviz
        ]);
      });
      shared-shell = base-shell [pkgs.boost175];
      static-shell = base-shell [
        pkgs.glibc.static
        (pkgs.boost175.override { enableShared = false; enabledStatic = true; })
      ];
  in {
    devShell = shared-shell;
    devShells = {
      shared = shared-shell;
      static = static-shell;
    };
  });
}
