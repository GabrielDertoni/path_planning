const path = require("path");
const CopyPlugin = require("copy-webpack-plugin");
const WasmPackPlugin = require("@wasm-tool/wasm-pack-plugin");

const dist = path.resolve(__dirname, "dist");

module.exports = {
  mode: "production",
  entry: {
    index: "./ts/index.ts"
  },
  // entry: "./bootstrap.js",
  output: {
    path: dist,
    filename: "[name].js"
  },
  devServer: {
    static: {
      directory: path.resolve(__dirname, "static"),
      staticOptions: {},
      watch: true,
    }
  },
  plugins: [
    new CopyPlugin([
      path.resolve(__dirname, "static")
    ]),

    new CopyPlugin([
      path.resolve(__dirname, "bootstrap.js")
    ]),

    new WasmPackPlugin({
      crateDirectory: __dirname,
      withTypeScript: true,
    }),
  ],
  resolve: { 
    extensions: [".ts", ".tsx", ".js", ".wasm"],
  },
  module: {
    rules: [{
      test: /\.tsx?$/,
      loader: "ts-loader"
    }]
  },
  experiments: {
    asyncWebAssembly: true
  }
};
