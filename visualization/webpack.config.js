const path = require("path");
const CopyPlugin = require("copy-webpack-plugin");
const WasmPackPlugin = require("@wasm-tool/wasm-pack-plugin");
const HtmlWebpackPlugin = require("html-webpack-plugin");

const dist = path.resolve(__dirname, "dist");

module.exports = {
  mode: "production",
  entry: path.resolve(__dirname, "./app/index.tsx"),
  resolve: { 
    extensions: [".ts", ".tsx", ".js", ".wasm"],
  },
  output: {
    path: dist,
    filename: "[name].js"
  },
  devServer: {
    static: {
      directory: path.resolve(__dirname, "public"),
      staticOptions: {},
      watch: true,
    }
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: "./public/index.html"
    }),

    new WasmPackPlugin({
      crateDirectory: __dirname,
      withTypeScript: true,
    }),
  ],
  module: {
    rules: [{
      test: /\.(js|ts)x?$/,
      exclude: /node_modules/,
      loader: "babel-loader"
    }]
  },
  experiments: {
    asyncWebAssembly: true
  }
};
