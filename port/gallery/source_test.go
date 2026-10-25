package gallery_test

import (
	"archive/zip"
	"os"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/stretchr/testify/require"
)

const fakeGalleryGoMod = "module " + gallery.Module + "\n\ngo 1.21\n"

// writeFile writes contents to path, making its directory first.
func writeFile(t *testing.T, path, contents string) {
	t.Helper()
	require.NoError(t, os.MkdirAll(filepath.Dir(path), 0o755))
	require.NoError(t, os.WriteFile(path, []byte(contents), 0o600))
}

// isolateGo points the go command at an empty module cache and a module proxy in a directory, so a test sees a
// fresh machine and needs no network. It returns the proxy directory.
func isolateGo(t *testing.T) string {
	t.Helper()
	root := t.TempDir()
	proxy := filepath.Join(root, "proxy")
	require.NoError(t, os.MkdirAll(proxy, 0o755))
	t.Setenv("GOMODCACHE", filepath.Join(root, "modcache"))
	t.Setenv("GOPROXY", "file://"+filepath.ToSlash(proxy))
	t.Setenv("GOSUMDB", "off")
	t.Setenv("GONOSUMDB", "")
	t.Setenv("GOPRIVATE", "")
	t.Setenv("GOWORK", "off")
	t.Setenv("GOTOOLCHAIN", "local")
	// The module cache is read-only by default, which t.TempDir cannot remove.
	t.Setenv("GOFLAGS", "-mod=mod -modcacherw")
	return proxy
}

// publish adds version of the gallery module, holding one model package, to the proxy.
func publish(t *testing.T, proxy, version, model string) {
	t.Helper()
	dir := filepath.Join(proxy, filepath.FromSlash(gallery.Module), "@v")
	writeFile(t, filepath.Join(dir, "list"), version+"\n")
	writeFile(t, filepath.Join(dir, version+".info"), `{"Version":"`+version+`","Time":"2026-01-01T00:00:00Z"}`)
	writeFile(t, filepath.Join(dir, version+".mod"), fakeGalleryGoMod)
	f, err := os.Create(filepath.Join(dir, version+".zip"))
	require.NoError(t, err)
	zw := zip.NewWriter(f)
	prefix := gallery.Module + "@" + version + "/"
	for name, contents := range map[string]string{
		"go.mod":                                fakeGalleryGoMod,
		"models/" + model + "/" + model + ".go": "package " + model + "\n",
	} {
		w, err := zw.Create(prefix + name)
		require.NoError(t, err)
		_, err = w.Write([]byte(contents))
		require.NoError(t, err)
	}
	require.NoError(t, zw.Close())
	require.NoError(t, f.Close())
}

// consumer writes a module that requires the gallery at version, with the extra go.mod lines, and returns its
// directory.
func consumer(t *testing.T, version, extra string) string {
	t.Helper()
	dir := t.TempDir()
	writeFile(t, filepath.Join(dir, "go.mod"),
		"module example.com/consumer\n\ngo 1.21\n\nrequire "+gallery.Module+" "+version+"\n"+extra)
	return dir
}

// TestLocateDownloadsAMissingModule requires Locate to fetch the pinned gallery into an empty module cache, as CI
// sees it before anything else ran.
func TestLocateDownloadsAMissingModule(t *testing.T) {
	proxy := isolateGo(t)
	publish(t, proxy, "v0.1.0", "fake")
	work := consumer(t, "v0.1.0", "")

	src, err := gallery.Locate(t.Context(), work, "fake")
	require.NoError(t, err)
	require.Equal(t, "v0.1.0", src.Version)
	_, err = os.Stat(filepath.Join(src.Dir, "fake.go"))
	require.NoError(t, err)

	_, err = gallery.Locate(t.Context(), work, "nosuchmodel")
	require.ErrorContains(t, err, "has no model nosuchmodel")
}

// TestLocateReportsAFailedDownload requires a clear message when the pinned version cannot be fetched.
func TestLocateReportsAFailedDownload(t *testing.T) {
	isolateGo(t)
	work := consumer(t, "v0.9.0", "")

	_, err := gallery.Locate(t.Context(), work, "fake")
	require.ErrorContains(t, err, gallery.Module)
}

// TestLocateRefusesAReplacedGallery requires Locate to refuse a replace directive: the code would come from the
// replacement while the generated header named the pinned version.
func TestLocateRefusesAReplacedGallery(t *testing.T) {
	isolateGo(t)
	local := t.TempDir()
	writeFile(t, filepath.Join(local, "go.mod"), fakeGalleryGoMod)
	writeFile(t, filepath.Join(local, "models", "fake", "fake.go"), "package fake\n")
	abs, err := filepath.Abs(local)
	require.NoError(t, err)
	work := consumer(t, "v0.1.0", "\nreplace "+gallery.Module+" => "+filepath.ToSlash(abs)+"\n")

	_, err = gallery.Locate(t.Context(), work, "fake")
	require.ErrorContains(t, err, "replace")
}
