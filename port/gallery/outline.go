package gallery

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"go/ast"
	"go/printer"
	"go/scanner"
	"go/token"
	"strings"
)

// SourceDigest returns a digest of the named functions' code. Comments and formatting do not change it; any change
// to a token of the code does. A fragment records the digest of the Go it was written against, so a changed gallery
// function forces a review of the fragment.
func (m *Model) SourceDigest(funcs ...string) (string, error) {
	h := sha256.New()
	for _, name := range funcs {
		fd, err := m.pkg.callable(name)
		if err != nil {
			return "", err
		}
		code, err := m.pkg.codeTokens(fd)
		if err != nil {
			return "", fmt.Errorf("digest %s: %w", name, err)
		}
		h.Write([]byte(name + "\n" + code + "\n"))
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}

// codeTokens renders the tokens of a declaration, one per line, without its comments, its line breaks or its
// spacing. A trailing comma and a semicolon before a closing bracket, which gofmt adds or drops with a line break,
// are left out, and every semicolon reads the same whether the source wrote it or a line break inserted it.
func (p *parsedPackage) codeTokens(n ast.Node) (string, error) {
	var src bytes.Buffer
	if err := printer.Fprint(&src, p.fset, n); err != nil {
		return "", err
	}
	type tok struct {
		t   token.Token
		lit string
	}
	var toks []tok
	var s scanner.Scanner
	fset := token.NewFileSet()
	var scanErr error
	s.Init(fset.AddFile("", -1, src.Len()), src.Bytes(), func(_ token.Position, msg string) {
		scanErr = fmt.Errorf("scan: %s", msg)
	}, 0)
	for {
		_, t, lit := s.Scan()
		if t == token.EOF {
			break
		}
		if t == token.SEMICOLON {
			lit = ";"
		}
		toks = append(toks, tok{t, lit})
	}
	if scanErr != nil {
		return "", scanErr
	}
	var sb strings.Builder
	for i, k := range toks {
		next := token.EOF
		if i+1 < len(toks) {
			next = toks[i+1].t
		}
		closing := next == token.RPAREN || next == token.RBRACE || next == token.RBRACK || next == token.EOF
		if (k.t == token.COMMA || k.t == token.SEMICOLON) && closing {
			continue
		}
		sb.WriteString(k.t.String())
		if k.lit != "" && k.lit != k.t.String() {
			sb.WriteString(" " + k.lit)
		}
		sb.WriteByte('\n')
	}
	return sb.String(), nil
}
