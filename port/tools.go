//go:build tools

// This file keeps the gallery requirement in go.mod. The port reads the gallery
// spur source from the module cache at the pinned revision; it never compiles it.
package port

import _ "github.com/lestrrat-3d/decad-gallery/models/spurgear"
